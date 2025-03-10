// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/PrintCommand.h>

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

  if (m_autonomousCommand) {
    m_autonomousCommand.value().get()->Schedule();
    //m_autonomousCommand.value()->Schedule();
    //m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

ctre::phoenix6::hardware::TalonFX winchMotor {WINCH_MOTOR_ID, "rio"};

ctre::phoenix6::controls::VoltageOut a {units::voltage::volt_t{0.0}};

configs::TalonFXConfiguration talonFXConfigs{};

configs::Slot0Configs slot0Configs = talonFXConfigs.Slot0;


// set Motion Magic settings
auto& motionMagicConfigs = talonFXConfigs.MotionMagic;

controls::MotionMagicTorqueCurrentFOC b{0_tr};

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand.value().get()->Cancel();
  }

  // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
  // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  // slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  // slot0Configs.kP = 0.2; // A position error of 2.5 rotations results in 12 V output
  // // slot0Configs.kI = 0; // no output for integrated error
  // // slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

  // motionMagicConfigs.MotionMagicCruiseVelocity = 10_tps; // Target cruise velocity of 80 rps
  // motionMagicConfigs.MotionMagicAcceleration = 160_tr_per_s_sq; // Target acceleration of 160 rps/s (0.5 seconds)
  // motionMagicConfigs.MotionMagicJerk = 1600_tr_per_s_cu; // Target jerk of 1600 rps/s/s (0.1 seconds)

  // winchMotor.GetConfigurator().Apply(talonFXConfigs);

  // winchMotor.SetPosition(100_tr);
}

void Robot::TeleopPeriodic() {
  // Swerve Calibration
  // frc::SmartDashboard::PutNumber("FL Angle", this->fl.GetAbsolutePosition().GetValueAsDouble());
  // frc::SmartDashboard::PutNumber("FR Angle", this->fr.GetAbsolutePosition().GetValueAsDouble());
  // frc::SmartDashboard::PutNumber("BL Angle", this->bl.GetAbsolutePosition().GetValueAsDouble());
  // frc::SmartDashboard::PutNumber("BR Angle", this->br.GetAbsolutePosition().GetValueAsDouble());

  
  // if (m_container.joystick.Cross().Get())
  // {
  //   winchMotor.SetControl(b.WithPosition(200_tr));
  // }
  
  // if (m_container.joystick.Square().Get())
  // {
  //   winchMotor.SetControl(b.WithPosition(0_tr));
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