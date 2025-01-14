#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <stdio.h>

void Robot::RobotInit() {
  // Define the auto options
  autoChooser.SetDefaultOption("Far Auto (0)", AUTO_FAR);
  autoChooser.AddOption("Near Auto (1)", AUTO_NEAR);
  frc::SmartDashboard::PutData("Auto Modes", &autoChooser);
}

void Robot::RobotPeriodic() {
  // Reads back the selected auto to the user
  frc::SmartDashboard::PutNumber("Selected Auto: ", autoChooser.GetSelected());
}

void Robot::AutonomousInit() {
  AutoType type = autoChooser.GetSelected();
  printf("Running auto %d\n", type);
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
