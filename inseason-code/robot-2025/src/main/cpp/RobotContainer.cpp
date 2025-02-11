// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/events/EventTrigger.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandPtr.h>

RobotContainer::RobotContainer()
{
  // Register named commands here
  this->pather = SwervePather(&this->drivetrain);

  pathplanner::NamedCommands::registerCommand("PrintHi", frc2::cmd::Print("Hey hi!"));

  pathplanner::EventTrigger("trigger").OnTrue(frc2::cmd::Print("Passed the trigger event"));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
  // Note that X is defined as forward according to WPILib convention,
  // and Y is defined as to the left according to WPILib convention.
  drivetrain.SetDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.ApplyRequest([this]() -> auto&& {
          return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
              .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
              .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
      })
  );

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
  // Set control needs to be run every frame to keep moving

  return frc2::cmd::Sequence(
    frc2::cmd::Print("Starting..."),
    frc2::cmd::RunOnce([this](){this->drivetrain.ResetPose(frc::Pose2d{});}),
    frc2::cmd::Run([this]() {
      return this->drivetrain.SetControl(this->drive.WithVelocityY(1_mps));
    }),
    // drivetrain.ApplyRequest([this]() -> auto&& {
    //       return drive.WithVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
    //           .WithVelocityY(0.5 * MaxSpeed) // Drive left with negative X (left)
    //           .WithRotationalRate(0 * MaxAngularRate); // Drive counterclockwise with negative X (left)
    // }),
    frc2::cmd::Print("Done with first, waiting..."),
    frc2::cmd::Wait(3_s),
    frc2::cmd::Print("Stopping..."),
    drivetrain.ApplyRequest([this]() -> auto&& {
          return drive.WithVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
              .WithVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
              .WithRotationalRate(0 * MaxAngularRate); // Drive counterclockwise with negative X (left)
      })
  );
}
