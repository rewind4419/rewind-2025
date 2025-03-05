// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

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
  drivetrain.SetDefaultCommand(
    drivetrain.ApplyRequest([this]() -> auto&& {
        return drive.WithVelocityX(-driver.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .WithVelocityY(-driver.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
            .WithRotationalRate(-driver.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
    })
  );
  
  // // reset the field-centric heading on left bumper press
  //driver.R2().OnTrue(frc2::cmd::Print("e"));
  // driver.Triangle().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

  winch.SetDefaultCommand(winch.HoldPos());

  //elevator.SetDefaultCommand(elevator.HoldPos());
  
  //coralarm.SetDefaultCommand(coralarm.HoldPos());

  coralarm.SetDefaultCommand(coralarm.SetPositionProvider([] () -> units::angle::turn_t {
    return units::angle::turn_t{
      frc::SmartDashboard::GetNumber("CoralArmPos", 0.0)
    };
  }));

  elevator.SetDefaultCommand(elevator.SetHeightProvider([] () -> units::angle::turn_t {
    return units::angle::turn_t{
      frc::SmartDashboard::GetNumber("ElevatorPos", 0.0)
    };
  }));

  // driver.Cross().OnTrue(coralarm.SetPosition(0_tr));
  // driver.Square().OnTrue(coralarm.SetPosition(0.1_tr));
  // driver.Triangle().OnTrue(coralarm.ResetPosition());
  // driver.Circle().OnTrue(coralarm.SetPosition(0.25_tr));

  // driver.Cross().OnTrue(elevator.SetHeight(0_tr));
  // driver.Square().OnTrue(elevator.SetHeight(1_tr));
  // driver.Triangle().OnTrue(elevator.Home());
  // driver.Circle().OnTrue(elevator.SetHeight(4_tr));

  // driver.R2().WhileTrue(winch.DrivePower([this]() -> float {
  //   return (driver.GetR2Axis() - driver.GetL2Axis()) * 0.5;
  // }));

  // driver.L2().WhileTrue(winch.DrivePower([this]() -> float {
  //   return (driver.GetR2Axis() - driver.GetL2Axis()) * 0.5;
  // }));

  // driver.Square().OnTrue(winch.GotoPosition(100_tr));
  // driver.Cross().OnTrue(winch.GotoPosition(0_tr));
  
  // winch.SetDefaultCommand(winch.DrivePower([this]() -> float {
  //   return (driver.GetR2Axis() - driver.GetL2Axis()) * 0.5;
  // }));
  //driver.Circle().WhileTrue(coralarm.CoralArmRun(0.5));

  // driver.Triangle().OnTrue(coralarm.CoralArmResetPosition());

  // driver.Cross().WhileTrue(coralarm.CoralArmTo(0_tr));
  // driver.Circle().WhileTrue(coralarm.CoralArmTo(-0.1_tr));

  // elevator.SetDefaultCommand(elevator.DrivePower([this]() -> float {
  //   return (driver.GetR2Axis() - driver.GetL2Axis()) * 0.5;
  // }));

  driver.Triangle().OnTrue(elevator.Home());

  // driver.Cross().WhileTrue(elevator.SetHeight(0.0));
  // driver.Circle().WhileTrue(elevator.SetHeight(3.0));
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  return this->pather.ResetPose(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_rad)))
    .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d{4_m, 3_m, frc::Rotation2d(0_rad)}))
    .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d{-1_m, 1_m, frc::Rotation2d(0_rad)}))
    .AndThen(this->pather.DriveWaypointSimple(frc::Pose2d{0_m, 0_m, frc::Rotation2d(0_rad)}));
  
  //return this->pather.DriveFor(10_s, 3_mps);
  
  // return frc2::cmd::Run([this] () {
  //   printf("Starting drive\n");
  //   this->drivetrain.SetControl(this->drive.WithVelocityY(5_mps));
  // });

  //return autoChooser.GetSelected();

}
