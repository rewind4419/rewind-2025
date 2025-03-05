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
}

void RobotContainer::ConfigureBindings()
{
  drivetrain.SetDefaultCommand(
    drivetrain.ApplyRequest([this]() -> auto&& {
        return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
            .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
    })
  );
  
  // // reset the field-centric heading on left bumper press
  //joystick.R2().OnTrue(frc2::cmd::Print("e"));
  // joystick.Triangle().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

  winch.SetDefaultCommand(winch.HoldPos());

  joystick.R2().WhileTrue(winch.DrivePower([this]() -> float {
    return (joystick.GetR2Axis() - joystick.GetL2Axis()) * 0.5;
  }));

  joystick.L2().WhileTrue(winch.DrivePower([this]() -> float {
    return (joystick.GetR2Axis() - joystick.GetL2Axis()) * 0.5;
  }));

  // joystick.Square().OnTrue(winch.GotoPosition(100_tr));
  // joystick.Cross().OnTrue(winch.GotoPosition(0_tr));
  
  // winch.SetDefaultCommand(winch.DrivePower([this]() -> float {
  //   return (joystick.GetR2Axis() - joystick.GetL2Axis()) * 0.5;
  // }));
  //joystick.Circle().WhileTrue(coralarm.CoralArmRun(0.5));

  // joystick.Triangle().OnTrue(coralarm.CoralArmResetPosition());

  // joystick.Cross().WhileTrue(coralarm.CoralArmTo(0_tr));
  // joystick.Circle().WhileTrue(coralarm.CoralArmTo(-0.1_tr));

  joystick.Triangle().OnTrue(elevator.Home());

  joystick.Cross().WhileTrue(elevator.SetHeight(1.0));
  joystick.Circle().WhileTrue(elevator.SetHeight(10.0));
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
