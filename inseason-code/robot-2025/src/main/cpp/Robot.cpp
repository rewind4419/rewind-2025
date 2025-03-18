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

frc::Field2d m_field;

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand.has_value()) {
    m_autonomousCommand.value().get()->Schedule();
    //m_autonomousCommand.value().get()->Schedule();
  }
  frc::SmartDashboard::PutData("Field", &m_field);
}

AutoManager m;


void Robot::AutonomousPeriodic() {
  photon::PhotonPipelineResult result = m.Cam1.GetLatestResult();
  bool hasTargets = result.HasTargets();
  photon::PhotonTrackedTarget target = result.GetBestTarget();
  std::optional<photon::EstimatedRobotPose> optional = m.Estimate(result);
  if (hasTargets){
    printf("Got a target, checking if it has a value\n");
    if (optional.has_value()){
      printf("I got a value!\n");
      m_field.SetRobotPose(optional.value().estimatedPose.ToPose2d());
    // photon::EstimatedRobotPose estimation = m.Estimate(result).value();
    frc::SmartDashboard::PutNumber("Photon pose X", optional.value().estimatedPose.X().value());
    frc::SmartDashboard::PutNumber("Photon pose Y", optional.value().estimatedPose.Y().value());
    frc::SmartDashboard::PutNumber("Photon pose Z", optional.value().estimatedPose.Z().value());
    } else {
      printf("No value here.   ");
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