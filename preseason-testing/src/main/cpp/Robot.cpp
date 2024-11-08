#pragma once

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include <stdio.h>

void Robot::RobotInit() {

}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();

    printf("Auto type%d\n", this->selectedAuto->GetDouble(-1));

    switch (this->selectedAuto->GetInteger(-1))
    {
    case -1:
      this->selectedAutoName->SetString("N/A - defaulting to Red Near");
      break;
    case 0:
      this->selectedAutoName->SetString("Red near");
      break;
    case 1:
      this->selectedAutoName->SetString("Blue near");
      break;
    case 2:
      this->selectedAutoName->SetString("Red far");
      break;
    }
}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {

  

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
