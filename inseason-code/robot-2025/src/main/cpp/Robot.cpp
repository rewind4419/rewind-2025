// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/PrintCommand.h>
#include <photon/PhotonUtils.h>
#include <subsystems/Auto.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/SwervePather.h"

#include "Config.h"

frc::Field2d m_field {};

frc::FieldObject2d* m_object;

AutoManager m;

Robot::Robot() {
  m.Init();
  frc::SmartDashboard::PutData("Field", &m_field);
  m_object = m_field.GetObject("bob");
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  //m_field.SetRobotPose(m_container.drivetrain.GetState().Pose);

  frc::Pose2d robotPose = m_container.drivetrain.GetState().Pose;

  // frc::Pose2d centerPose {m.aprilTagFieldLayout.GetFieldLength() * 0.5, m.aprilTagFieldLayout.GetFieldWidth() * 0.5, frc::Rotation2d {0_rad}};

  // frc::Pose2d flippedRobotPose {centerPose.X() - (r obotPose.X() - centerPose.X()), centerPose.Y() - (robotPose.Y() - centerPose.Y()), robotPose.Rotation() + frc::Rotation2d {M_PI * 1_rad}};

  m_field.SetRobotPose(robotPose);
  m_object->SetPose(SwapFieldSide(robotPose));
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  m.updRoutine();
  if (m.optional.has_value()){
    //printf("Yes, going to %f, %f\n", m.optional.value().estimatedPose.X().value(), m.optional.value().estimatedPose.Y().value());
    m_container.drivetrain.AddVisionMeasurement(m.optional.value().estimatedPose.ToPose2d(), utils::GetCurrentTime());
  }
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand.has_value()) {
    m_autonomousCommand.value().get()->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  if (m_container.autoVisionEnabled)
  {
    m.updRoutine();
    if (m.optional.has_value()){
      //printf("Yes, going to %f, %f\n", m.optional.value().estimatedPose.X().value(), m.optional.value().estimatedPose.Y().value());
      m_container.drivetrain.AddVisionMeasurement(m.optional.value().estimatedPose.ToPose2d(), utils::GetCurrentTime());
    }
  }
}

void Robot::AutonomousExit() {}

ctre::phoenix6::hardware::TalonFX winchMotor {WINCH_MOTOR_ID, "rio"};

ctre::phoenix6::controls::VoltageOut a {units::voltage::volt_t{0.0}};

configs::TalonFXConfiguration talonFXConfigs{};

configs::Slot0Configs slot0Configs = talonFXConfigs.Slot0;


// set Motion Magic settings
auto& motionMagicConfigs = talonFXConfigs.MotionMagic;

controls::MotionMagicTorqueCurrentFOC b{0_tr};

void Robot::TeleopInit() {
  if (m_autonomousCommand.has_value()) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {
  // Swerve Calibration
  // frc::SmartDashboard::PutNumber("FL Angle", this->fl.GetAbsolutePosition().GetValueAsDouble());
  // frc::SmartDashboard::PutNumber("FR Angle", this->fr.GetAbsolutePosition().GetValueAsDouble());
  // frc::SmartDashboard::PutNumber("BL Angle", this->bl.GetAbsolutePosition().GetValueAsDouble());
  // frc::SmartDashboard::PutNumber("BR Angle", this->br.GetAbsolutePosition().GetValueAsDouble());
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
  
}
void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

