// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <optional>

#include "util/Controller.h"
#include "RobotContainer.h"

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc2/command/CommandPtr.h>

enum AutoType
{
  AUTO_FAR,
  AUTO_NEAR
};

class Robot : public frc::TimedRobot {
public:

  frc::SendableChooser<AutoType> autoChooser;
  AutoType m_autoSelected;

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

  std::optional<frc2::CommandPtr> m_autonomousCommand;
  RobotContainer m_container;
};
