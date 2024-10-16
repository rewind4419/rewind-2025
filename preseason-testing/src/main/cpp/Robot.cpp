#include "Robot.h"

#include "subsystems/Drivetrain.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
    Drivetrain d = Drivetrain();
}

void Robot::TeleopPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
