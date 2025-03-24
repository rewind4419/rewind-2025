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

#include <frc/smartdashboard/Field2d.h>

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

  // frc::SmartDashboard::PutNumber("Surgery Ramp Rate", 0.0);
  // frc::SmartDashboard::PutNumber("Surgery Rot Ramp Rate", 0.0);

  autoVisionEnabled = false;
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
      // if (driver.R1().Get())
      // {
      //   return drive.WithVelocityX(-deadzone(driver.GetLeftY(), 0.1) * SurgeryModeSpeed) // Drive forward with negative Y (forward)
      //       .WithVelocityY(-deadzone(driver.GetLeftX(), 0.1) * SurgeryModeSpeed) // Drive left with negative X (left)
      //       .WithRotationalRate(-deadzone(driver.GetRightX(), 0.1) * SurgeryModeAngularRate); // Drive counterclockwise with negative X (left)
      // } else {
      //   return drive.WithVelocityX(-deadzone(driver.GetLeftY(), 0.1) * MaxSpeed) // Drive forward with negative Y (forward)
      //       .WithVelocityY(-deadzone(driver.GetLeftX(), 0.1) * MaxSpeed) // Drive left with negative X (left)
      //       .WithRotationalRate(-deadzone(driver.GetRightX(), 0.1) * MaxAngularRate); // Drive counterclockwise with negative X (left)
      // }
      if (driver.R1().Get())
      {
        CurrentSpeed = CurrentSpeed + clamp(SurgeryModeSpeed - CurrentSpeed, -0.25_mps, 0.25_mps);
        CurrentAngularRate = CurrentAngularRate + clamp(SurgeryModeAngularRate - CurrentAngularRate, -0.2_rad_per_s, 0.2_rad_per_s);
      } else {
        CurrentSpeed = CurrentSpeed + clamp(MaxSpeed - CurrentSpeed, -0.25_mps, 0.25_mps);
        CurrentAngularRate = CurrentAngularRate + clamp(MaxAngularRate - CurrentAngularRate, -0.2_rad_per_s, 0.2_rad_per_s); 
      }

      frc::SmartDashboard::PutNumber("Current Speed", CurrentSpeed.value());
      frc::SmartDashboard::PutNumber("Current Angular Rate", CurrentAngularRate.value());

      return drive.WithVelocityX(-deadzone(driver.GetLeftY(), 0.1) * CurrentSpeed) // Drive forward with negative Y (forward)
            .WithVelocityY(-deadzone(driver.GetLeftX(), 0.1) * CurrentSpeed) // Drive left with negative X (left)
            .WithRotationalRate(-deadzone(driver.GetRightX(), 0.1) * CurrentAngularRate); // Drive counterclockwise with negative X (left)
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
      return mate.GetRightY() * -0.1_tr;
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

  //Resets the Robot
  mate.Cross().OnTrue(
    frc2::cmd::Select<int>(
      [this] {
        printf("Running Cross!\n");
        if (robotState.currentState == STATE_DELIVER_LOW)
        {return 0;}
        return 1;

        
      },
      std::pair{0,
        // Currently in STATE_DELIVER_LOW
        robotState.SetCurrentState(STATE_NEUTRAL)
          .AndThen(robotState.SetDeliverHeight(DELIVER_ZERO))

          .AndThen(coralwrist.SetPosition(CORAL_WRIST_DELIVER, false))
          .AndThen(frc2::cmd::Deadline(frc2::cmd::Wait(1.0_s), coralarm.CoralArmRunIntake(-10_tps)))

          

          .AndThen(coralarm.SetPosition(CORAL_ARM_SAFE))
          .AndThen(coralwrist.SetPosition(CORAL_WRIST_FUNNEL))
          .AndThen(elevator.SetHeight(ELEVATOR_MIN))
          .AndThen(coralarm.SetPosition(CORAL_ARM_MIN))
      }
    ).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
  );

  // mate.Touchpad().OnTrue(
  //   frc2::cmd::Select<int>(
  //     [this] {
  //       if (robotState.currentState == STATE_NEUTRAL)
  //       {
  //         std::cout << "Current state is NEUTRAL" << std::endl;
  //         return 0;
  //       }
  //       std::cout << "Current state is not NEUTRAL" << std::endl;
  //       return 1;
  //     },
  //     std::pair{0,
  //     printf("Running Touchpad\n");
  //     coralarm.SetPosition(CORAL_ARM_CLIMB)
  //     .AndThen(coralwrist.SetPosition(CORAL_WRIST_CLIMB));
  //     }
  //   )
  //   .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
  // );

  mate.POVRight().OnTrue(winch.SetFlipperPosition(0.3_tr));
  mate.POVLeft().OnTrue(winch.SetFlipperPosition(0.0_tr));

  //Extends the arm out
  mate.Touchpad().OnTrue(
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
          .AndThen(coralarm.SetPosition(CORAL_ARM_CLIMB))
          .AlongWith(coralwrist.SetPosition(CORAL_WRIST_CLIMB))
          .AndThen(elevator.SetHeight(ELEVATOR_MIN))
          .AndThen(winch.SetFlipperPosition(0.25_tr))
      }
    )
    //.WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
    .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf)
  ); 

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
          .AndThen(coralarm.SetPosition(CORAL_ARM_EXTENDED))
          .AlongWith(coralwrist.SetPosition(CORAL_WRIST_EXTENDED))
          .AndThen(elevator.SetHeight(ELEVATOR_MIN))
      }
    )
    //.WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
    .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
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
          .AndThen(elevator.SetHeight(0.7_tr)) //1.3_tr
      }
    )
    .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
  );

  // //Cancels current command
  // mate.Share().OnTrue(frc2::cmd::RunOnce([this] {
  //   elevator.GetCurrentCommand()->Cancel();
  // }));

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
  
  mate.R2().WhileTrue(coralarm.CoralArmRunIntake(CORAL_ARM_INTAKE_SPEED)); //intake
  mate.L2().WhileTrue(coralarm.CoralArmRunIntake(CORAL_ARM_OUTTAKE_SPEED)); //outake

  mate.Options().OnTrue(
    frc2::cmd::RunOnce([this] {
      elevator.GetCurrentCommand()->Cancel();
      coralarm.GetCurrentCommand()->Cancel();
    })
  );

  // Recovery Mode
  mate.Share().WhileTrue(
    frc2::cmd::RunOnce([this] {
      printf("Running recovery\n");
      
      elevator.target = elevator.motor1.GetPosition().GetValue();
      coralarm.target = coralarm.motor1.GetPosition().GetValue();
      this->robotState.currentState = STATE_NEUTRAL;
    })
    .AndThen(elevator.SetHeightProvider([this] {
        printf("Incrementing elevator by %f, now at %f\n", mate.GetRightY(), elevator.target.value());
        elevator.target += (mate.GetRightY()) * 0.03_tr;
        return elevator.target;
      }, true, true).AlongWith(coralarm.SetPositionProvider([this] {
        printf("Incrementing arm by %f, now at %f\n", mate.GetLeftY(), coralarm.target.value());
        coralarm.target += (mate.GetLeftY()) * 0.01_tr;
        return coralarm.target;
      }, true, true)))
    .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
    // .Until([this] {
    //   bool optionVal = mate.Circle().Get();
    //   if (optionVal) {printf("Circle pressed! ending\n"); return true;}
    //   return false;
    // })
  );

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
// On Red
// Left Side Human Player: 16.898, 1.342, 2.205

// In front of Tag 6:
// 14.286, 3.2579, 2.1289 - scoring on right tree

// In front of Tag 11, left tree
// 12.6723, 2.7138, 1.032

// In front of tag 11, right tree (better)
// 12.941693, 2.544800, 1.034950

// Red starting left side
// 9.667, 1.450, -0.044

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
  autoChooser.SetDefaultOption("No Auto", [this] () -> frc2::CommandPtr {
    return frc2::cmd::Print("Ran the no auto");
  });

  // autoChooser.AddOption("Multipoint Auto", [this] () -> frc2::CommandPtr {
  //   return this->pather.DriveWaypointSimple(frc::Pose2d {16_m, 6.5_m, frc::Rotation2d{3.14_rad}}, 2.5_mps, 0.5_m, 0.5_rad, 2.0, 1.0)
  //     .AndThen(this->pather.LockWheels())
  //     .AndThen(frc2::cmd::Wait(1.0_s))
  //     .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d {16_m, 3.6_m, frc::Rotation2d{3.14_rad}}, 3.0_mps, 1.0_m, 0.5_rad, 2.0, 100.0))
  //     .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d {14.23_m, 2.0_m, frc::Rotation2d{2.0_rad}}, 3.0_mps, 0.1_m, 0.2_rad, 100.0, 1.0))
  //     .AndThen(this->pather.LockWheels())
  //   ;
  // });

  autoChooser.AddOption("Drive Straight Forward, Place L4", [this] () -> frc2::CommandPtr {
    return 
      robotState.SetCurrentState(STATE_DELIVER_LOW)
      .AndThen(coralarm.SetPosition(CORAL_ARM_EXTENDED).AlongWith(coralwrist.SetPosition(CORAL_WRIST_EXTENDED)))
      
      .AndThen(elevator.SetHeight(ELEVATOR_MIN))
      .AndThen(robotState.SetDeliverHeight(DELIVER_HIGH))
      .AndThen(elevator.SetHeightProvider([this] {
        return robotState.GetDeliverHeight();
      }, false))
      // Drive up to score
      .AndThen(pather.DriveFor(4_s, 1_mps))
      .AndThen(pather.LockWheels())
      .AndThen(frc2::cmd::Wait(0.1_s))
      .AndThen(coralarm.SetPosition(0.4_tr, false))
      .AndThen(frc2::cmd::Wait(0.1_s))
      .AndThen(coralwrist.SetPosition(0.2_tr, false))
      .AndThen(frc2::cmd::Deadline(frc2::cmd::Wait(2.0_s), coralarm.CoralArmRunIntake(-10_tps)))
      .AndThen(pather.DriveFor(0.5_s, -1_mps))
      .AndThen(coralarm.SetPosition(CORAL_ARM_SAFE))
      .AndThen(coralwrist.SetPosition(CORAL_WRIST_FUNNEL))
      .AndThen(frc2::cmd::Wait(1_s))
      .AndThen(elevator.SetHeight(ELEVATOR_MIN))
      .AndThen(robotState.SetCurrentState(STATE_NEUTRAL))
      .AndThen(robotState.SetDeliverHeight(DELIVER_ZERO))
      .AndThen(coralarm.SetPosition(CORAL_ARM_MIN))
    ;
  });

  // DONT UNCOMMENT THIS, theres a missing parenthesis somewhere and it has a compile error
  // autoChooser.AddOption("RED One Piece Back L2", [this] () -> frc2::CommandPtr {
  //   return 
  //     //frc2::cmd::Wait(5.0_s)

  //     // Drive 1m away from scoring pose
  //     pather.DriveWaypointSimple(
  //       frc::Pose2d{10.7169736_m,3.38677531_m, frc::Rotation2d {1.034950_rad}}, 2.0_mps, 0.05_m, 0.2_rad, 2.0, 3.0
      
  //     ).AndThen(pather.LockWheels()).AlongWith(
  //       robotState.SetCurrentState(STATE_DELIVER_LOW)
  //       .AndThen(coralarm.SetPosition(CORAL_ARM_EXTENDED).AlongWith(coralwrist.SetPosition(CORAL_WRIST_EXTENDED)))
        
  //       .AndThen(elevator.SetHeight(ELEVATOR_MIN))
  //       .AndThen(robotState.SetDeliverHeight(DELIVER_LOW))
  //       .AndThen(elevator.SetHeightProvider([this] {
  //         return robotState.GetDeliverHeight();
  //       }, false))
  //     ))
  //     .AndThen(frc2::cmd::RunOnce([this] {autoVisionEnabled = true;}))
  //     .AndThen(frc2::cmd::Wait(0.5_s))
  //     .AndThen(frc2::cmd::RunOnce([this] {autoVisionEnabled = false;}))
  //     // Drive up to score
  //     .AndThen(pather.DriveWaypointSimple(
  //       frc::Pose2d{11.7169736_m,3.38677531_m, frc::Rotation2d {1.034950_rad}}, 1.0_mps, 0.05_m, 0.05_rad, 2.0, 3.0
  //     ))
  //     .AndThen(pather.LockWheels())
  //     .AndThen(frc2::cmd::Wait(0.1_s))
  //     .AndThen(coralarm.SetPosition(0.4_tr, false))
  //     .AndThen(frc2::cmd::Wait(0.1_s))
  //     .AndThen(coralwrist.SetPosition(0.2_tr, false))
  //     .AndThen(frc2::cmd::Deadline(frc2::cmd::Wait(1.0_s), coralarm.CoralArmRunIntake(-10_tps)))
  //     .AndThen(coralarm.SetPosition(CORAL_ARM_SAFE))
  //     .AndThen(coralwrist.SetPosition(CORAL_WRIST_FUNNEL))
  //     .AndThen(pather.DriveWaypointSimple(
  //      frc::Pose2d{10.7169736_m,3.38677531_m, frc::Rotation2d {1.034950_rad}}, 2.0_mps, 0.05_m, 0.2_rad, 2.0, 3.0
  //     ))
  //     .AndThen(elevator.SetHeight(ELEVATOR_MIN))
  //     .AndThen(robotState.SetCurrentState(STATE_NEUTRAL))
  //     .AndThen(robotState.SetDeliverHeight(DELIVER_ZERO))
  //     .AndThen(coralarm.SetPosition(CORAL_ARM_MIN))
  //   ;
  // });

  autoChooser.AddOption("RED One Piece Far Left L2", [this] () -> frc2::CommandPtr {
    return 
      //frc2::cmd::Wait(5.0_s)
      pather.DriveWaypointSimple(
        frc::Pose2d{11.29_m,1.24_m, frc::Rotation2d {-0.044_rad}}, 2_mps, 0.2_m, 0.2_rad, 2.0, 3.0
      )
      // Drive 1m away from scoring pose
      .AndThen(pather.DriveWaypointSimple(
        frc::Pose2d{12.441693_m,1.6787746_m, frc::Rotation2d {1.034950_rad}}, 2.0_mps, 0.05_m, 0.2_rad, 2.0, 3.0
      ).AndThen(pather.LockWheels()).AlongWith(
        robotState.SetCurrentState(STATE_DELIVER_LOW)
        .AndThen(coralarm.SetPosition(CORAL_ARM_EXTENDED).AlongWith(coralwrist.SetPosition(CORAL_WRIST_EXTENDED)))
        
        .AndThen(elevator.SetHeight(ELEVATOR_MIN))
        .AndThen(robotState.SetDeliverHeight(DELIVER_LOW))
        .AndThen(elevator.SetHeightProvider([this] {
          return robotState.GetDeliverHeight();
        }, false))
      ))
      .AndThen(frc2::cmd::RunOnce([this] {autoVisionEnabled = true;}))
      .AndThen(frc2::cmd::Wait(0.5_s))
      .AndThen(frc2::cmd::RunOnce([this] {autoVisionEnabled = false;}))
      // Drive up to score
      .AndThen(pather.DriveWaypointSimple(
        frc::Pose2d{12.941693_m, 2.544800_m, frc::Rotation2d {1.034950_rad}}, 1.0_mps, 0.05_m, 0.05_rad, 2.0, 3.0
      ))
      .AndThen(pather.LockWheels())
      .AndThen(frc2::cmd::Wait(0.1_s))
      .AndThen(coralarm.SetPosition(0.4_tr, false))
      .AndThen(frc2::cmd::Wait(0.1_s))
      .AndThen(coralwrist.SetPosition(0.2_tr, false))
      .AndThen(frc2::cmd::Deadline(frc2::cmd::Wait(1.0_s), coralarm.CoralArmRunIntake(-10_tps)))
      .AndThen(coralarm.SetPosition(CORAL_ARM_SAFE))
      .AndThen(coralwrist.SetPosition(CORAL_WRIST_FUNNEL))
      .AndThen(pather.DriveWaypointSimple(
       frc::Pose2d{12.441693_m,1.6787746_m, frc::Rotation2d {1.034950_rad}}, 2.0_mps, 0.05_m, 0.2_rad, 2.0, 3.0
      ))
      .AndThen(elevator.SetHeight(ELEVATOR_MIN))
      .AndThen(robotState.SetCurrentState(STATE_NEUTRAL))
      .AndThen(robotState.SetDeliverHeight(DELIVER_ZERO))
      .AndThen(coralarm.SetPosition(CORAL_ARM_MIN))
      .AndThen(pather.DriveWaypointSimple(
       frc::Pose2d{16.898_m, 1.342_m, frc::Rotation2d {2.205_rad}}, 3.0_mps, 0.05_m, 0.2_rad, 2.0, 1.0
      ))
    ;
  });

  autoChooser.AddOption("RED One Piece Far Right L2", [this] () -> frc2::CommandPtr {
    return 
      pather.DriveWaypointSimple(
        MirrorLongWays(frc::Pose2d{11.29_m,1.24_m, frc::Rotation2d {-0.044_rad}}), 2_mps, 0.2_m, 0.2_rad, 2.0, 3.0
      )
      // Drive 1m away from scoring pose
      .AndThen(pather.DriveWaypointSimple(
        MirrorLongWays(frc::Pose2d{12.1723_m, 1.8477746_m, frc::Rotation2d {1.034950_rad}}), 2.0_mps, 0.05_m, 0.2_rad, 2.0, 3.0
      ).AndThen(pather.LockWheels()).AlongWith(
        robotState.SetCurrentState(STATE_DELIVER_LOW)
        .AndThen(coralarm.SetPosition(CORAL_ARM_EXTENDED).AlongWith(coralwrist.SetPosition(CORAL_WRIST_EXTENDED)))
        
        .AndThen(elevator.SetHeight(ELEVATOR_MIN))
        .AndThen(robotState.SetDeliverHeight(DELIVER_LOW))
        .AndThen(elevator.SetHeightProvider([this] {
          return robotState.GetDeliverHeight();
        }, false))
      ))
      .AndThen(frc2::cmd::RunOnce([this] {autoVisionEnabled = true; printf("Enabling auto vision\n");}))
      .AndThen(frc2::cmd::Wait(0.5_s))
      .AndThen(frc2::cmd::RunOnce([this] {autoVisionEnabled = false; printf("Disabling auto vision\n");}))
      // Drive up to score
      .AndThen(pather.DriveWaypointSimple(
        MirrorLongWays(frc::Pose2d{12.6723_m, 2.7138_m, frc::Rotation2d {1.034950_rad}}), 1.0_mps, 0.05_m, 0.05_rad, 2.0, 3.0
      ))
      .AndThen(pather.LockWheels())
      .AndThen(frc2::cmd::Wait(0.1_s))
      .AndThen(coralarm.SetPosition(0.4_tr, false))
      .AndThen(frc2::cmd::Wait(0.1_s))
      .AndThen(coralwrist.SetPosition(0.2_tr, false))
      .AndThen(frc2::cmd::Deadline(frc2::cmd::Wait(1.0_s), coralarm.CoralArmRunIntake(-10_tps)))
      .AndThen(coralarm.SetPosition(CORAL_ARM_SAFE))
      .AndThen(coralwrist.SetPosition(CORAL_WRIST_FUNNEL))
      .AndThen(pather.DriveWaypointSimple(
        MirrorLongWays(frc::Pose2d{12.1723_m, 1.8477746_m, frc::Rotation2d {1.034950_rad}}), 2.0_mps, 0.05_m, 0.2_rad, 2.0, 3.0
      ))
      .AndThen(elevator.SetHeight(ELEVATOR_MIN))
      .AndThen(robotState.SetCurrentState(STATE_NEUTRAL))
      .AndThen(robotState.SetDeliverHeight(DELIVER_ZERO))
      .AndThen(coralarm.SetPosition(CORAL_ARM_MIN))
      .AndThen(pather.DriveWaypointSimple(
       MirrorLongWays(frc::Pose2d{16.898_m, 1.342_m, frc::Rotation2d {2.205_rad}}), 3.0_mps, 0.05_m, 0.2_rad, 2.0, 1.0
      ))
    ;
  });

  autoChooser.AddOption("Emergency Taxi", [this] () -> frc2::CommandPtr {
    return 
      pather.DriveFor(2_s, 1_mps)
    ;
  });

  autoChooser.AddOption("One Piece Close Right L3 (sketchy)", [this] () -> frc2::CommandPtr { //"One Piece Close Right L3 (untested)"
    return 
      pather.DriveWaypointSimple(
        frc::Pose2d{11.052824_m, 4.566236_m, frc::Rotation2d {-0.204760_rad}}, 3.0_mps, 0.5_m, 2.0_rad, 2.0, 100.0
      )
      .AndThen(pather.DriveWaypointSimple(
        frc::Pose2d{11.942582_m, 6.385897_m, frc::Rotation2d {-1.57470_rad}}, 3.0_mps, 0.5_m, 2.0_rad, 100.0, 100.0
      ))
      .AndThen(pather.DriveWaypointSimple(
        frc::Pose2d{15.586_m, 6.10892_m, frc::Rotation2d {-2.379003_rad}}, 3.0_mps, 0.5_m, 2.0_rad, 100.0, 4.0
      ))
      // Second Piece Score
      .AndThen(pather.DriveWaypointSimple(
        frc::Pose2d{15.414805_m, 4.710612_m, frc::Rotation2d {-3.103120_rad}}, 3.0_mps, 0.05_m, 0.2_rad, 2.0, 1.0
      ).AndThen(pather.LockWheels()).AlongWith(
        robotState.SetCurrentState(STATE_DELIVER_LOW)
        .AndThen(coralarm.SetPosition(CORAL_ARM_EXTENDED).AlongWith(coralwrist.SetPosition(CORAL_WRIST_EXTENDED)))
        
        .AndThen(elevator.SetHeight(ELEVATOR_MIN))
        .AndThen(robotState.SetDeliverHeight(DELIVER_MID))
        .AndThen(elevator.SetHeightProvider([this] {
          return robotState.GetDeliverHeight();
        }, false))
      ))
      .AndThen(frc2::cmd::Wait(0.1_s))
      .AndThen(pather.DriveWaypointSimple(
        frc::Pose2d{14.414805_m, 4.710612_m, frc::Rotation2d {-3.103120_rad}}, 1.0_mps, 0.05_m, 0.05_rad, 2.0, 3.0
      ))
      .AndThen(pather.LockWheels())
      .AndThen(frc2::cmd::Wait(0.1_s))
      .AndThen(coralarm.SetPosition(0.4_tr, false))
      .AndThen(frc2::cmd::Wait(0.1_s))
      .AndThen(coralwrist.SetPosition(0.2_tr, false))
      .AndThen(frc2::cmd::Deadline(frc2::cmd::Wait(1.0_s), coralarm.CoralArmRunIntake(-10_tps)))
      .AndThen(coralarm.SetPosition(CORAL_ARM_SAFE))
      .AndThen(coralwrist.SetPosition(CORAL_WRIST_FUNNEL))
      .AndThen(pather.DriveWaypointSimple(
       frc::Pose2d{15.414805_m, 4.710612_m, frc::Rotation2d {-3.103120_rad}}, 3_mps, 0.2_m, 0.2_rad, 5.0, 3.0
      ))
      .AndThen(elevator.SetHeight(ELEVATOR_MIN))
      .AndThen(robotState.SetCurrentState(STATE_NEUTRAL))
      .AndThen(robotState.SetDeliverHeight(DELIVER_ZERO))
      .AndThen(coralarm.SetPosition(CORAL_ARM_MIN))
    ;
  });

  // autoChooser.AddOption("2 Piece Close Right L2 L3 (DONT RUN, ITS WAY OVER 15 SECONDS)", [this] () -> frc2::CommandPtr {
  //   return 
  //     pather.DriveWaypointSimple(
  //       frc::Pose2d{11.052824_m, 4.566236_m, frc::Rotation2d {-0.204760_rad}}, 3.0_mps, 0.5_m, 2.0_rad, 2.0, 100.0
  //     )
  //     .AndThen(pather.DriveWaypointSimple(
  //       frc::Pose2d{11.942582_m, 6.385897_m, frc::Rotation2d {-1.57470_rad}}, 3.0_mps, 0.5_m, 2.0_rad, 100.0, 100.0
  //     ))
  //     .AndThen(pather.DriveWaypointSimple(
  //       frc::Pose2d{15.586_m, 6.10892_m, frc::Rotation2d {-2.379003_rad}}, 3.0_mps, 0.5_m, 2.0_rad, 100.0, 100.0
  //     ))
  //     .AndThen(
  //       pather.DriveWaypointSimple(
  //         frc::Pose2d{15.414805_m, 4.710612_m, frc::Rotation2d {-3.103120_rad}}, 3.0_mps, 0.05_m, 0.2_rad, 100.0, 3.0
  //       ).AndThen(pather.LockWheels()).AlongWith(
  //         robotState.SetCurrentState(STATE_DELIVER_LOW)
  //         .AndThen(coralarm.SetPosition(CORAL_ARM_EXTENDED).AlongWith(coralwrist.SetPosition(CORAL_WRIST_EXTENDED)))
          
  //         .AndThen(elevator.SetHeight(ELEVATOR_MIN))
  //         .AndThen(robotState.SetDeliverHeight(DELIVER_MID))
  //         .AndThen(elevator.SetHeightProvider([this] {
  //           return robotState.GetDeliverHeight();
  //         }, false))
  //       ))
  //     .AndThen(pather.DriveWaypointSimple(
  //       frc::Pose2d{14.414805_m, 4.710612_m, frc::Rotation2d {-3.103120_rad}}, 1.5_mps, 0.05_m, 0.05_rad, 2.0, 3.0
  //     ))
  //     .AndThen(pather.LockWheels())
  //     .AndThen(frc2::cmd::Wait(0.1_s))
  //     .AndThen(coralarm.SetPosition(0.4_tr, false))
  //     .AndThen(frc2::cmd::Wait(0.1_s))
  //     .AndThen(coralwrist.SetPosition(0.2_tr, false))
  //     .AndThen(frc2::cmd::Deadline(frc2::cmd::Wait(0.5_s), coralarm.CoralArmRunIntake(-10_tps)))
  //     .AndThen(coralarm.SetPosition(CORAL_ARM_SAFE, false))
  //     .AndThen(coralwrist.SetPosition(CORAL_WRIST_FUNNEL, false))
  //     .AndThen(pather.DriveWaypointSimple(
  //       frc::Pose2d{15.414805_m, 4.710612_m, frc::Rotation2d {-3.103120_rad}}, 3_mps, 0.5_m, 0.2_rad, 5.0, 3.0
  //     ).AndThen(pather.DriveWaypointSimple(
  //       frc::Pose2d{16.410535_m, 7.045300_m, frc::Rotation2d {-2.289_rad}}, 3.0_mps, 0.05_m, 0.1_rad, 2.0, 1.0
  //     )).AlongWith(
  //       elevator.SetHeight(ELEVATOR_MIN)
  //       .AndThen(robotState.SetCurrentState(STATE_NEUTRAL))
  //       .AndThen(robotState.SetDeliverHeight(DELIVER_ZERO))
  //       .AndThen(coralarm.SetPosition(CORAL_ARM_MIN))
  //     )
  //     // Go to human player, drive forward, and inatke
  //     ).AndThen(pather.DriveFor(1_s, -0.2_mps).AlongWith(
  //       elevator.SetHeight(0.8_tr)
  //       .AndThen(coralwrist.SetPosition(0.03_tr))
  //     ))
  //     .AndThen(pather.LockWheels())
  //     .AndThen(frc2::cmd::Deadline(frc2::cmd::Wait(2.0_s), coralarm.CoralArmRunIntake(-5_tps)))
  //     .AndThen(frc2::cmd::Deadline(frc2::cmd::Wait(1.0_s), coralarm.CoralArmRunIntake(10_tps)))
  //     .AndThen(elevator.SetHeight(ELEVATOR_MIN))
  //     .AndThen(coralwrist.SetPosition(CORAL_WRIST_FUNNEL))

  //     // Second Piece Score
  //     .AndThen(pather.DriveWaypointSimple(
  //       frc::Pose2d{15.414805_m, 4.710612_m, frc::Rotation2d {-3.103120_rad}}, 4.0_mps, 0.05_m, 0.2_rad, 2.0, 2.0
  //     ).AndThen(pather.LockWheels()).AlongWith(
  //       robotState.SetCurrentState(STATE_DELIVER_LOW)
  //       .AndThen(coralarm.SetPosition(CORAL_ARM_EXTENDED).AlongWith(coralwrist.SetPosition(CORAL_WRIST_EXTENDED)))
        
  //       .AndThen(elevator.SetHeight(ELEVATOR_MIN))
  //       .AndThen(robotState.SetDeliverHeight(DELIVER_LOW))
  //       .AndThen(elevator.SetHeightProvider([this] {
  //         return robotState.GetDeliverHeight();
  //       }, false))
  //     ))
  //     .AndThen(frc2::cmd::Wait(0.1_s))
  //     .AndThen(pather.DriveWaypointSimple(
  //       frc::Pose2d{14.414805_m, 4.710612_m, frc::Rotation2d {-3.103120_rad}}, 1.0_mps, 0.05_m, 0.05_rad, 2.0, 3.0
  //     ))
  //     .AndThen(pather.LockWheels())
  //     .AndThen(frc2::cmd::Wait(0.1_s))
  //     .AndThen(coralarm.SetPosition(0.4_tr, false))
  //     .AndThen(frc2::cmd::Wait(0.1_s))
  //     .AndThen(coralwrist.SetPosition(0.2_tr, false))
  //     .AndThen(frc2::cmd::Deadline(frc2::cmd::Wait(1.0_s), coralarm.CoralArmRunIntake(-10_tps)))
  //     .AndThen(coralarm.SetPosition(CORAL_ARM_SAFE))
  //     .AndThen(coralwrist.SetPosition(CORAL_WRIST_FUNNEL));
  //     //.AndThen(pather.DriveWaypointSimple(
  //     //  frc::Pose2d{15.414805_m, 4.710612_m, frc::Rotation2d {-3.103120_rad}}, 3_mps, 0.2_m, 0.2_rad, 5.0, 3.0
  //     //.AndThen(elevator.SetHeight(ELEVATOR_MIN))
  //     //.AndThen(robotState.SetCurrentState(STATE_NEUTRAL))
  //     //.AndThen(robotState.SetDeliverHeight(DELIVER_ZERO))
  //     //.AndThen(coralarm.SetPosition(CORAL_ARM_MIN))
  //   ;
  // });

  frc::SmartDashboard::PutData(&autoChooser);
}
