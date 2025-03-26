#include "Robot.h"

#include "Config.h"



#include "util/maths.h"

#include "Hardware.h"

Robot::Robot() {
    
    frc::SmartDashboard::PutData("Field", &m_field);
    m_object = m_field.GetObject("bob");

    frc::SmartDashboard::PutNumber("Elevator SetPos", 0.0);
}

void Robot::RobotPeriodic() {
    
}

void Robot::TeleopInit() {
    queue.Clear();
}

// swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
//     .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors

void Robot::TeleopPeriodic() {
    // drivetrain.SetControl(
    //     drive
    //     .WithVelocityX(deadzone(driver.GetLeftY(), 0.1) * 5.7_mps)
    //     .WithVelocityY(deadzone(-driver.GetLeftX(), 0.1) * 5.7_mps)
    //     .WithRotationalRate(deadzone(driver.GetRightX(), 0.1) * 0.75_rad_per_s)
    // );

    if (driver.GetCrossButtonPressed())
    {
        printf("Cross\n");
        // queue.AddTask(new MotorPositionTask(hardware.elevator, 1.0, true, 0.15));
        queue.AddTask(new MotorPositionTask(hardware.elevator, 0.0));
    }

    if (driver.GetTriangleButtonPressed())
    {
        printf("Triangle\n");
        queue.AddTask(new MotorPositionTask(hardware.elevator, 1.0));
    }

    if (driver.GetSquareButtonPressed())
    {
        printf("Square\n");
        hardware.elevator->targetPosition = frc::SmartDashboard::GetNumber("Elevator SetPos", 0.0);
        printf("Setting elevator to %f\n", hardware.elevator->targetPosition);
    }

    frc::SmartDashboard::PutNumber("Elevator GetPos", hardware.elevator->motor.GetPosition().GetValueAsDouble());

    queue.Update();
    hardware.Update();
}

void Robot::TeleopExit() {}

void Robot::AutonomousInit() {
    queue.Clear();

    queue.AddTask(new CustomTask([] {
        printf("Starting auto\n");
        return true;
    }));
}

void Robot::AutonomousPeriodic() {



    queue.Update();
    hardware.Update();
}

void Robot::AutonomousExit() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {
    // Swerve Calibration
    // frc::SmartDashboard::PutNumber("FL Angle", this->fl.GetAbsolutePosition().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("FR Angle", this->fr.GetAbsolutePosition().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("BL Angle", this->bl.GetAbsolutePosition().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("BR Angle", this->br.GetAbsolutePosition().GetValueAsDouble());
}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

