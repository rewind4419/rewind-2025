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
  //this->pather = SwervePather(&this->drivetrain);

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


  /*drivetrain.ApplyRequest([this]() -> auto&& {
          return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
              .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
              .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
      })*/

  // elevator.SetDefaultCommand(elevator.RunOnce([] () {printf("Elevator default\n");}));

  // joystick.R1().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
  // joystick.L1().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
  //     return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
  // }));


  

  // // Run SysId routines when holding back/start and X/Y.
  // // Note that each routine should be run exactly once in a single log.
  // (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
  // (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
  // (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  
  // reset the field-centric heading on left bumper press

  joystick.Triangle().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

  // joystick.Cross().OnTrue(drivetrain.RunOnce([this] {
  //   drivetrain.ResetRotation(frc::Rotation2d {0.0_rad});
  // }));

  joystick.Square().OnTrue(elevator.SetHeight(0.5f));
  joystick.Circle().OnTrue(elevator.SetHeight(0.0f));

  // drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  
  // frc2::SequentialCommandGroup()

  return this->pather.DriveFor(10_s, 3_mps);
  
  // return frc2::cmd::Run([this] () {
  //   printf("Starting drive\n");
  //   this->drivetrain.SetControl(this->drive.WithVelocityY(5_mps));
  // });

  //return autoChooser.GetSelected();

}
