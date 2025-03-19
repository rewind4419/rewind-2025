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
#include <frc/SmartDashboard/SmartDashboard.h>

#include "Config.h"

Robot::Robot() {}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand.has_value()) {
    m_autonomousCommand.value().get()->Schedule();
    //m_autonomousCommand.value().get()->Schedule();
  }
}

AutoManager m;


void Robot::AutonomousPeriodic() {
  // m.updRoutine();
  // if (m.optional.has_value()){
  //   //printf("Yes, going to %f, %f\n", m.optional.value().estimatedPose.X().value(), m.optional.value().estimatedPose.Y().value());
  //   m_container.drivetrain.AddVisionMeasurement(m.optional.value().estimatedPose.ToPose2d(), utils::GetCurrentTime());
  // }
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

  // m.updRoutine();
  // if (m.optional.has_value()){
  //   //printf("Yes, timestamp: %f\n", m.optional.value().timestamp.value());
  //   //m_container.drivetrain.AddVisionMeasurement(m.optional.value().estimatedPose.ToPose2d(), m.optional.value().timestamp);
  //   m_container.drivetrain.AddVisionMeasurement(m.optional.value().estimatedPose.ToPose2d(), utils::GetCurrentTime());
  // }
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

