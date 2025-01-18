#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Command.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/StartEndCommand.h>
#include <stdio.h>

/*
Tasks

- Drivetrain subsystem (waiting for a test drivetrain to run configurator)

- Photon Vision Code

- Elevator Subsystem that uses commands to set height

*/

void Robot::RobotInit() {
  // Define the auto options
  autoChooser.SetDefaultOption("Far Auto (0)", AUTO_FAR);
  autoChooser.AddOption("Near Auto (1)", AUTO_NEAR);
  frc::SmartDashboard::PutData("Auto Modes", &autoChooser);
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  // Reads back the selected auto to the user
  frc::SmartDashboard::PutNumber("Selected Auto: ", autoChooser.GetSelected());
}

void Robot::AutonomousInit() {
  AutoType type = autoChooser.GetSelected();
  printf("Running auto %d\n", type);

  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand.has_value())
  {
    m_autonomousCommand.value().Schedule();
  }
}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {

}

void Robot::TestInit()
{

}

void Robot::TestPeriodic()
{

}

void Robot::DisabledInit()
{

}

void Robot::DisabledPeriodic()
{

}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
