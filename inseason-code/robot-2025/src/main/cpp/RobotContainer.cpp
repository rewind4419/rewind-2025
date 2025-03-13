// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

RobotContainer::RobotContainer()
{
  // Register named commands here
  // this->pather = SwervePather(&this->drivetrain);
  // Another option that allows you to specify the default auto by its name
  // autoChooser = AutoBuilder::buildAutoChooser("My Default Auto");

  //frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

  ConfigureBindings();

  frc::SmartDashboard::PutNumber("ElevatorPos", 0.0);
  frc::SmartDashboard::PutNumber("CoralArmPos", 0.0);
}

void RobotContainer::ConfigureBindings()
{
  // Drivetrain //
  drivetrain.SetDefaultCommand(
    drivetrain.ApplyRequest([this]() -> auto&& {
      if (driver.R1().Get())
      {
        // if (lenSq(driver.GetLeftY()+driver.GetLeftX()) > 0.05*0.05)
        // {}
        // Surgery Mode
        return drive.WithVelocityX(-driver.GetLeftY() * SurgeryModeSpeed) // Drive forward with negative Y (forward)
            .WithVelocityY(-driver.GetLeftX() * SurgeryModeSpeed) // Drive left with negative X (left)
            .WithRotationalRate(-driver.GetRightX() * SurgeryModeAngularRate); // Drive counterclockwise with negative X (left)
      } else {
        return drive.WithVelocityX(-driver.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .WithVelocityY(-driver.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
            .WithRotationalRate(-driver.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
      }
    })
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

  elevator.SetDefaultCommand(elevator.HoldPos());
  coralarm.SetDefaultCommand(coralarm.HoldPos());

  // coralarm.SetDefaultCommand(coralarm.CoralArmBaseIntake(0.01));
  // // elevator.SetDefaultCommand(elevator.HoldPos());
  // // coralarm.SetDefaultCommand(coralarm.HoldPos());

  // Mate Controls

  mate.Circle().OnTrue(
    frc2::cmd::Select<int>(
      [this] {
        printf("Running circle!\n");
        if (robotState.targetState == STATE_DELIVER_LOW && robotState.currentState == STATE_DELIVER_LOW)
        {return 0;}
        if (robotState.targetState == STATE_FUNNEL && robotState.currentState == STATE_FUNNEL)
        {return 1;}

        
      },
      std::pair{0,
        robotState.SetTargetState(STATE_NEUTRAL)
          .AndThen(coralarm.SetPosition(CORAL_ARM_SAFE))
          .AndThen(elevator.SetHeight(ELEVATOR_MIN))
          .AndThen(coralarm.SetPosition(CORAL_ARM_MIN))
          .AndThen(robotState.SetCurrentState(STATE_NEUTRAL))
      },
      std::pair{1,
        robotState.SetTargetState(STATE_NEUTRAL)
          .AndThen(coralarm.SetPosition(CORAL_ARM_MIN))
          .AndThen(elevator.SetHeight(ELEVATOR_MIN))
          .AndThen(robotState.SetCurrentState(STATE_NEUTRAL))
      }
    ).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
  );

  mate.Triangle().OnTrue(
    frc2::cmd::Select<int>(
      [this] {
        if (robotState.targetState == STATE_NEUTRAL && robotState.currentState == STATE_NEUTRAL)
        {return 0;}

        return 1;
      },
      std::pair{0, 
        robotState.SetTargetState(STATE_DELIVER_LOW)
          .AndThen(coralarm.SetPosition(CORAL_ARM_SAFE))
          .AndThen(elevator.SetHeight(2_tr))
          .AndThen(robotState.SetCurrentState(STATE_DELIVER_LOW))
      }
    )
    .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
  );

  mate.Square().OnTrue(
    frc2::cmd::Select<int>(
      [this] {
        if (robotState.targetState == STATE_NEUTRAL && robotState.currentState == STATE_NEUTRAL)
        {return 0;}

        return 1;
      },
      std::pair{0, 
        robotState.SetTargetState(STATE_FUNNEL)
          .AndThen(elevator.SetHeight(1.5_tr))
          .AndThen(robotState.SetCurrentState(STATE_FUNNEL))
      }
    )
    .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
  );

  mate.Share().OnTrue(frc2::cmd::RunOnce([this] {
    elevator.GetCurrentCommand()->Cancel();
  }));

  mate.POVUp().OnTrue(
    elevator.SetHeightProvider([this] {return clamp(elevator.target + 1_tr, ELEVATOR_MIN, ELEVATOR_SAFE_MAX);})
    .Unless([this] {return (robotState.targetState != STATE_DELIVER_LOW) || (robotState.currentState != STATE_DELIVER_LOW);})
  );

  mate.POVDown().OnTrue(
    elevator.SetHeightProvider([this] {return clamp(elevator.target - 1_tr, ELEVATOR_MIN, ELEVATOR_SAFE_MAX);})
    .Unless([this] {return (robotState.targetState != STATE_DELIVER_LOW) || (robotState.currentState != STATE_DELIVER_LOW);})
  );
  
  mate.POVLeft().OnTrue(
    elevator.SetHeightProvider([this] {return clamp(elevator.target - 0.1_tr, ELEVATOR_MIN, ELEVATOR_SAFE_MAX);})
    .Unless([this] {return (robotState.targetState != STATE_DELIVER_LOW) || (robotState.currentState != STATE_DELIVER_LOW);})
  );

  mate.POVRight().OnTrue(
    elevator.SetHeightProvider([this] {return clamp(elevator.target + 0.1_tr, ELEVATOR_MIN, ELEVATOR_SAFE_MAX);})
    .Unless([this] {return (robotState.targetState != STATE_DELIVER_LOW) || (robotState.currentState != STATE_DELIVER_LOW);})
  );

  mate.R1().WhileTrue(winch.DrivePower([this]() -> float {
    return 0.5f;
  }));

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
  //     frc::SmartDashboard::GetNumber("CoralArmPos", 0.0)
  //   };
  // }));
  // elevator.SetDefaultCommand(elevator.SetHeightProvider([] () -> units::angle::turn_t {
  //   return units::angle::turn_t{
  //     frc::SmartDashboard::GetNumber("ElevatorPos", 0.0)
  //   };
  // }));
  
  driver.POVUp().OnTrue(winch.TestCommand());
  
  // driver.POVDown().OnTrue(winch.TestCommand2().Unless([this] {return winch.iscool;}));

  // driver.POVDown().OnTrue(
  //   frc2::cmd::Select<int>(
  //     [this] {
  //       if (1)
  //       {return 0;}

  //       return 1;
  //     },
  //     std::pair{0, 
  //       winch.TestCommand2().Unless([this] {return winch.iscool;})
  //     }
  //   )
  // );

  // mate.Triangle().WhileTrue(coralarm.CoralArmRunIntake(1.0));
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  return this->pather.ResetPose(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_rad)))
    .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d{4_m, 3_m, frc::Rotation2d(0_rad)}))
    .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d{-1_m, 1_m, frc::Rotation2d(0_rad)}))
    .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d{0_m, 0_m, frc::Rotation2d(0_rad)}));
}
