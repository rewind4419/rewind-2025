// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandPS4Controller.h>

#include <frc/smartdashboard/SendableChooser.h>

#include "subsystems/Elevator.h"
#include "subsystems/SwerveDrivetrain.h"
#include "subsystems/SwervePather.h"
#include "subsystems/CoralArm.h"
#include "subsystems/Winch.h"
#include "subsystems/CoralWrist.h"

#include "subsystems/RobotState.h"

#include <ctre/phoenix6/CANcoder.hpp>

//#include <pathplanner/lib/config/RobotConfig.h>

class RobotContainer {
private:
  units::meters_per_second_t SurgeryModeSpeed = TunerConstants::kSpeedAt12Volts * 0.35;
  units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
  units::meters_per_second_t CurrentSpeed = TunerConstants::kSpeedAt12Volts;
  units::radians_per_second_t SurgeryModeAngularRate = 0.75_tps * 0.25;
  units::radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity
  units::radians_per_second_t CurrentAngularRate = 0.75_tps;

  /* Setting up bindings for necessary control of the swerve drive platform */
  swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
    //.WithDeadband(MaxSpeed * 0.05).WithRotationalDeadband(MaxAngularRate * 0.04) // Add a 10% deadband
    .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
  
  swerve::requests::SwerveDriveBrake brake{};
  swerve::requests::PointWheelsAt point{};

  // /* Note: This must be constructed before the drivetrain, otherwise we need to
  //  *       define a destructor to un-register the telemetry from the drivetrain */
  // Telemetry logger{MaxSpeed};

  frc::SendableChooser<std::function<frc2::CommandPtr()>> autoChooser;

  void AddAutos();
public:
  frc2::CommandPS4Controller driver{0};
  frc2::CommandPS4Controller mate{1};

  CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};
  SwervePather pather{&drivetrain};
  Elevator elevator {};
  CoralArm coralarm {};
  Winch winch;
  CoralWrist coralwrist;

  RobotState robotState;

  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  // TODO: add a robot config here
  //pathplanner::RobotConfig config {}s

  /*
  FL Encoder - 8
  FR Encoder - 9
  BL Encoder - 10
  BR Encoder - 11
   */

  bool autoVisionEnabled = false;
private:
  void ConfigureBindings();
};
