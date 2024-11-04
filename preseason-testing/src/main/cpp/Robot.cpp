#pragma once

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include <stdio.h>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
  printf("%f, %f, %f\n%d\n", this->driver.GetLeftX(), this->driver.GetLeftY(), this->driver.GetRightX(), this->driver.GetPOV());
  

  this->drivetrain.Update();
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
