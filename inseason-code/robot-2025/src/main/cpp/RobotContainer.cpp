// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <iostream>
#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include <frc/geometry/Rotation2d.h>

#include <frc/smartdashboard/SmartDashboard.h>

// #include <pathplanner/lib/auto/AutoBuilder.h>
// #include <pathplanner/lib/path/PathPlannerPath.h>
// #include <pathplanner/lib/commands/PathPlannerAuto.h>
// #include <pathplanner/lib/auto/NamedCommands.h>
// #include <pathplanner/lib/events/EventTrigger.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "util/maths.h"
#include <math.h>

RobotContainer::RobotContainer()
{
  // Register named commands here
  // this->pather = SwervePather(&this->drivetrain);
  // Another option that allows you to specify the default auto by its name
  // autoChooser = AutoBuilder::buildAutoChooser("My Default Auto");

  //frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

  ConfigureBindings();

  AddAutos();

  frc::SmartDashboard::PutNumber("Elevator Manual Position", 0.0);
  frc::SmartDashboard::PutNumber("Coral Arm Manual Position", 0.0);
  frc::SmartDashboard::PutNumber("Wrist Manual Position", 0.0);
}

double deadzone(double x, double deadzoneMax)
{
  if (abs(x) < deadzoneMax)
  {
    return 0.0;
  }

  return clamp(abs(x)-deadzoneMax, 0.0, 1.0-deadzoneMax) / (1.0 - deadzoneMax) * (x > 0.0 ? 1.0 : -1.0);
}

void RobotContainer::ConfigureBindings()
{
  
  

  // Drivetrain //
  drivetrain.SetDefaultCommand(
    drivetrain.ApplyRequest([this]() -> auto&& {
      if (driver.R1().Get())
      {
        return drive.WithVelocityX(-deadzone(driver.GetLeftY(), 0.1) * SurgeryModeSpeed) // Drive forward with negative Y (forward)
            .WithVelocityY(-deadzone(driver.GetLeftX(), 0.1) * SurgeryModeSpeed) // Drive left with negative X (left)
            .WithRotationalRate(-deadzone(driver.GetRightX(), 0.1) * SurgeryModeAngularRate); // Drive counterclockwise with negative X (left)
      } else {
        return drive.WithVelocityX(-deadzone(driver.GetLeftY(), 0.1) * MaxSpeed) // Drive forward with negative Y (forward)
            .WithVelocityY(-deadzone(driver.GetLeftX(), 0.1) * MaxSpeed) // Drive left with negative X (left)
            .WithRotationalRate(-deadzone(driver.GetRightX(), 0.1) * MaxAngularRate); // Drive counterclockwise with negative X (left)
      }
    })

    // drivetrain.Run([this] {
    //   frc::SmartDashboard::PutNumber("DriverXRaw", driver.GetLeftX());
    //   frc::SmartDashboard::PutNumber("DriverXDeadzoned", deadzone(driver.GetLeftX(), 0.1));
    // })
  );

  // // reset the field-centric heading on left bumper press
  driver.Triangle().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

  driver.Square().OnTrue(drivetrain.RunOnce([this] { 
    drivetrain.ResetRotation(drivetrain.GetOperatorForwardDirection() + frc::Rotation2d{90_deg});
  }));

  driver.Circle().OnTrue(drivetrain.RunOnce([this] { 
    drivetrain.ResetRotation(drivetrain.GetOperatorForwardDirection() - frc::Rotation2d{90_deg});
  }));

  driver.Cross().OnTrue(drivetrain.RunOnce([this] { 
    drivetrain.ResetRotation(drivetrain.GetOperatorForwardDirection() - frc::Rotation2d{180_deg});
  }));

  // Hold Pos (defaults)

  winch.SetDefaultCommand(winch.HoldPos());
  coralwrist.SetDefaultCommand(coralwrist.HoldPos([this] () -> units::angle::turn_t {
    // Coral Wrist offset
    if (robotState.currentState == STATE_DELIVER_LOW) {
      return mate.GetRightY() * -0.2_tr;
    }
    else {return 0_tr;}
  }));
  elevator.SetDefaultCommand(elevator.HoldPos());
  coralarm.SetDefaultCommand(coralarm.HoldPos());

  // Mate Controls

  //Resets the Robot
  mate.Circle().OnTrue(
    frc2::cmd::Select<int>(
      [this] {
        printf("Running circle!\n");
        if (robotState.currentState == STATE_DELIVER_LOW)
        {return 0;}
        if (robotState.currentState == STATE_FUNNEL)
        {return 1;}

        
      },
      std::pair{0,
        // Currently in STATE_DELIVER_LOW
        robotState.SetCurrentState(STATE_NEUTRAL)
          .AndThen(robotState.SetDeliverHeight(DELIVER_ZERO))
          .AndThen(coralarm.SetPosition(CORAL_ARM_SAFE))
          .AndThen(coralwrist.SetPosition(CORAL_WRIST_FUNNEL))
          .AndThen(elevator.SetHeight(ELEVATOR_MIN))
          .AndThen(coralarm.SetPosition(CORAL_ARM_MIN))
      },
      std::pair{1,
        // Currently in STATE_FUNNEL
        robotState.SetCurrentState(STATE_NEUTRAL)
          .AndThen(coralarm.SetPosition(CORAL_ARM_MIN))
          .AndThen(elevator.SetHeight(ELEVATOR_MIN))
          .AndThen(coralwrist.SetPosition(CORAL_WRIST_FUNNEL, true))
      }
    ).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
  );


  // // This code needs to be put in a command
  // // Currently, it only runs once when the robot turns on, so it wont work
  // if (mate.GetLeftY() > 0 || mate.GetLeftY() < 0) {
  //   coralwrist.SetPositionProvider ([] () -> units::angle::turn_t {
  //     return units::angle::turn_t{
  //       frc::SmartDashboard::GetNumber("CoralArmPos", 0.0)}
  //   ;});
  //   coralwrist.SetPosition(units::angle::turn_t{mate.GetLeftY()});
  // } else {
  //   coralwrist.SetPosition(units::angle::turn_t{0.0});
  // }

  //Extends the arm out
  mate.Triangle().OnTrue(
    frc2::cmd::Select<int>(
      [this] {
        if (robotState.currentState == STATE_NEUTRAL)
        {
          std::cout << "Current state is NEUTRAL" << std::endl;
          return 0;
        }
        std::cout << "Current state is not NEUTRAL" << std::endl;
        return 1;
      },
      std::pair{0, 
        robotState.SetCurrentState(STATE_DELIVER_LOW)
          .AndThen(coralarm.SetPosition(CORAL_ARM_SAFE))
          .AndThen(elevator.SetHeight(ELEVATOR_MIN))
      }
    )
    //.WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
    .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf)
  );
  
  //Going to funnel
  mate.Square().OnTrue(
    frc2::cmd::Select<int>(
      [this] {
        if (robotState.currentState == STATE_NEUTRAL)
        {return 0;}

        return 1;
      },
      std::pair{0, 
        robotState.SetCurrentState(STATE_FUNNEL)
          .AndThen(coralwrist.SetPosition(CORAL_WRIST_FUNNEL, true))
          .AndThen(elevator.SetHeight(1.5_tr))
      }
    )
    .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
  );

  //Cancels current command
  mate.Share().OnTrue(frc2::cmd::RunOnce([this] {
    elevator.GetCurrentCommand()->Cancel();
  }));

  //Moves the elevator up 1 level
  mate.POVUp().OnTrue(
    robotState.IncrementDeliverHeight()
    .AndThen(elevator.SetHeightProvider([this] {
      return robotState.GetDeliverHeight();
    }, false))
    .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf)
    .Unless([this] {return robotState.currentState != STATE_DELIVER_LOW;})
  );
  
  //Moves the elevator down 1 level
  mate.POVDown().OnTrue(
    robotState.DecrementDeliverHeight()
    .AndThen(elevator.SetHeightProvider([this] {
      return robotState.GetDeliverHeight();
    }, false))
    .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf)
    .Unless([this] {return robotState.currentState != STATE_DELIVER_LOW;})
  );

//   mate.POVUp().OnTrue(
//     elevator.SetHeightProvider([this] {return clamp(elevator.target + 1_tr, ELEVATOR_MIN, ELEVATOR_SAFE_MAX);})
//     .Unless([this] {return (robotState.currentState != STATE_DELIVER_LOW);})
//   );

//   mate.POVDown().OnTrue(
//     elevator.SetHeightProvider([this] {return clamp(elevator.target - 1_tr, ELEVATOR_MIN, ELEVATOR_SAFE_MAX);})
//     .Unless([this] {return (robotState.currentState != STATE_DELIVER_LOW);})
//   );
  
//   mate.POVLeft().OnTrue(
//     elevator.SetHeightProvider([this] {return clamp(elevator.target - 0.1_tr, ELEVATOR_MIN, ELEVATOR_SAFE_MAX);})
//     .Unless([this] {return (robotState.currentState != STATE_DELIVER_LOW);})
//   );

//   mate.POVRight().OnTrue(
//     elevator.SetHeightProvider([this] {return clamp(elevator.target + 0.1_tr, ELEVATOR_MIN, ELEVATOR_SAFE_MAX);})
//     .Unless([this] {return (robotState.currentState != STATE_DELIVER_LOW);})
//   );

  //Right bumper pulls climber upwards
  mate.R1().WhileTrue(winch.DrivePower([this]() -> float {
    return 0.5f;
  }));

  //Left bumper pulls climber downwards
  mate.L1().WhileTrue(winch.DrivePower([this]() -> float {
    return -0.5f;
  }));
  
  mate.R2().WhileTrue(coralarm.CoralArmRunIntake(10_tps)); //intake
  mate.L2().WhileTrue(coralarm.CoralArmRunIntake(-10_tps)); //outake

  // driver.Square().OnTrue(winch.GotoPosition(100_tr));
  // driver.Cross().OnTrue(winch.GotoPosition(0_tr));

  // Tests

  // // For smartdashboard control, comment the normal default task and uncomment these
  // coralarm.SetDefaultCommand(coralarm.SetPositionProvider([] () -> units::angle::turn_t {
  //   return units::angle::turn_t{
  //     frc::SmartDashboard::GetNumber("Coral Arm Manual Position", 0.0)
  //   };
  // }));
  // elevator.SetDefaultCommand(elevator.SetHeightProvider([] () -> units::angle::turn_t {
  //   return units::angle::turn_t{
  //     frc::SmartDashboard::GetNumber("Elevator Manual Position", 0.0)
  //   };
  // }));

  // coralwrist.SetDefaultCommand(coralwrist.SetPositionProvider([] () -> units::angle::turn_t {
  //   return units::angle::turn_t{
  //     frc::SmartDashboard::GetNumber("Wrist Manual Position", 0.0)
  //   };
  // }));
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  return (autoChooser.GetSelected())();
  

  // return this->pather.ResetPose(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_rad)))
  //   .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d{4_m, 3_m, frc::Rotation2d(0_rad)}))
  //   .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d{-1_m, 1_m, frc::Rotation2d(0_rad)}))
  //   .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d{0_m, 0_m, frc::Rotation2d(0_rad)}));
}

void RobotContainer::AddAutos()
{    
  autoChooser.SetDefaultOption("Print Auto", [this] () -> frc2::CommandPtr {
    return frc2::cmd::Print("Print auto ran");
  });

  autoChooser.AddOption("Test Auto", [this] () -> frc2::CommandPtr {
    return this->pather.ResetPose(frc::Pose2d {0_m, 0_m, frc::Rotation2d {0_rad}})
      .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d {1_m, 0_m, frc::Rotation2d{1_rad}}, 1_mps, 0.2));
  });

  autoChooser.AddOption("Multipoint Auto", [this] () -> frc2::CommandPtr {
    return this->pather.ResetPose(frc::Pose2d {0_m, 0_m, frc::Rotation2d {0_rad}})
      .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d {1_m, 0_m, frc::Rotation2d{0_rad}}, 1_mps, 0.2))
      .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d {1_m, -1_m, frc::Rotation2d{0_rad}}, 1_mps, 0.2))
      .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d {0_m, -1_m, frc::Rotation2d{0_rad}}, 1_mps, 0.2))
      .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d {0_m, 0_m, frc::Rotation2d{2_rad}}, 1_mps, 0.2))
    ;
  });

  frc::SmartDashboard::PutData(&autoChooser);
}
