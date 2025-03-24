#include "Robot.h"

#include "Config.h"

#include "queue/taskQueue.hpp"
#include "queue/standardTasks.hpp"

#include "SwerveConstants.h"
#include "SwerveDrivetrain.h"

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PS4Controller.h>

#include "util/maths.h"

frc::Field2d m_field {};
frc::FieldObject2d* m_object;

TaskQueue queue {};

SwerveDrivetrain drivetrain {TunerConstants::CreateDrivetrain()};

frc::PS4Controller driver {0};
frc::PS4Controller mate {1};

Robot::Robot() {
    frc::SmartDashboard::PutData("Field", &m_field);
    m_object = m_field.GetObject("bob");


}

void Robot::RobotPeriodic() {
    queue.update();
}

void Robot::DisabledInit() {

}

void Robot::DisabledPeriodic() {

}

void Robot::DisabledExit() {

}

void Robot::AutonomousInit() {
    queue.addTask(new CustomTask([] {
        printf("Starting auto\n");
        return true;
    }));
}

void Robot::AutonomousPeriodic() {

}

void Robot::AutonomousExit() {

}

void Robot::TeleopInit() {
  
}

swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
    //.WithDeadband(MaxSpeed * 0.05).WithRotationalDeadband(MaxAngularRate * 0.04) // Add a 10% deadband
    .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
  

void Robot::TeleopPeriodic() {
    // Swerve Calibration
    // frc::SmartDashboard::PutNumber("FL Angle", this->fl.GetAbsolutePosition().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("FR Angle", this->fr.GetAbsolutePosition().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("BL Angle", this->bl.GetAbsolutePosition().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("BR Angle", this->br.GetAbsolutePosition().GetValueAsDouble());

    drivetrain.SetControl(
        drive
        .WithVelocityX(deadzone(driver.GetLeftY(), 0.1) * 5.7_mps)
        .WithVelocityY(deadzone(-driver.GetLeftX(), 0.1) * 5.7_mps)
        .WithRotationalRate(deadzone(driver.GetRightX(), 0.1) * 0.75_rad_per_s)
    );
}

void Robot::TeleopExit() {

}

void Robot::TestInit() {

}
void Robot::TestPeriodic() {

}

void Robot::TestExit() {

}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

