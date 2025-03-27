#include "Robot.h"

#include "Config.h"

#include "util/maths.h"

#include "Hardware.h"
#include "State.h"

Robot::Robot() {
    
    frc::SmartDashboard::PutData("Field", &m_field);
    m_object = m_field.GetObject("bob");

    frc::SmartDashboard::PutNumber("Elevator SetPos", 0.0);
    
    
}

void Robot::RobotPeriodic() {
    frc::SmartDashboard::PutNumber("Wrist Pos", hardware.wrist->motor.GetPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Elevator Pos", hardware.elevator->motor.GetPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Arm Pos", hardware.arm->motor.GetPosition().GetValueAsDouble());

    frc::SmartDashboard::PutNumber("CurrentState", stateManager.currentState);
    frc::SmartDashboard::PutNumber("TargetState", stateManager.targetState);
}

void Robot::TeleopInit() {
    queue.Clear();
    hardware.winch->SetTargetPosition(hardware.winch->motor.GetPosition().GetValueAsDouble());
}

void Robot::TeleopPeriodic() {
    if (!driver.GetR1Button())
    {
        drivetrain.SetControl(
            drive
            .WithVelocityX(-deadzone(driver.GetLeftY(), 0.1) * 5.7_mps * 2)
            .WithVelocityY(-deadzone(driver.GetLeftX(), 0.1) * 5.7_mps * 2)
            .WithRotationalRate(-deadzone(driver.GetRightX(), 0.1) * 0.75_rad_per_s * 2)
        );
    }
    else
    {
        // Slow mode
        drivetrain.SetControl(
            drive
            .WithVelocityX(-deadzone(driver.GetLeftY(), 0.1) * 5.7_mps * 0.35 * 2)
            .WithVelocityY(-deadzone(driver.GetLeftX(), 0.1) * 5.7_mps * 0.35 * 2)
            .WithRotationalRate(-deadzone(driver.GetRightX(), 0.1) * 0.75_rad_per_s * 0.25 * 2)
        );
    }

    if (driver.GetTriangleButtonPressed())
    {
        drivetrain.SeedFieldCentric();
    }

    

    if (mate.GetR1Button())
    {
        hardware.winch->SetMode(CTRL_VOLTAGE);
        hardware.winch->targetVoltage = -6;
        hardware.winch->targetPosition = hardware.winch->motor.GetPosition().GetValueAsDouble();
    }
    else if (mate.GetL1Button())
    {
        hardware.winch->SetMode(CTRL_VOLTAGE);
        hardware.winch->targetVoltage = 6;
        hardware.winch->targetPosition = hardware.winch->motor.GetPosition().GetValueAsDouble();
    }
    else
    {
        hardware.winch->SetMode(CTRL_PID_POSITION);
    }

    hardware.intake->SetTargetVelocity(CORAL_ARM_INTAKE_SPEED * deadzone(mate.GetR2Axis() * 0.5 + 0.5, 0.1) + CORAL_ARM_OUTTAKE_SPEED * deadzone(mate.GetL2Axis() * 0.5 + 0.5, 0.1));

    if (mate.GetCircleButtonPressed())
    {
        if (stateManager.targetState == STATE_DELIVER)
        {
            queue.AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_SAFE, true, hardware.armDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_FUNNEL, true, hardware.wristDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, true, hardware.elevatorDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN, true, hardware.armDefaultEpsilon));
            queue.AddTask(new CurrentStateTask(STATE_NEUTRAL, &stateManager));
        }
        else if (stateManager.targetState == STATE_FUNNEL)
        {
            queue.AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN, true, hardware.armDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, true, hardware.elevatorDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_FUNNEL, true, hardware.wristDefaultEpsilon));
            queue.AddTask(new CurrentStateTask(STATE_NEUTRAL, &stateManager));
        }
        else if (stateManager.targetState == STATE_CLIMB)
        {
            queue.AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.flipper, FLIPPER_RETRACTED));
            queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_FUNNEL));
            queue.AddTask(new CurrentStateTask(STATE_NEUTRAL, &stateManager));
        }
    }

    if (mate.GetTriangleButtonPressed())
    {
        if (stateManager.targetState == STATE_NEUTRAL)
        {
            queue.AddTask(new TargetStateTask(STATE_DELIVER, &stateManager));
            queue.AddTask(new CustomTask([this] {stateManager.height = HEIGHT_ZERO; return true;}));
            queue.AddTask(new ForkTask(
                new MotorPositionTask(hardware.arm, CORAL_ARM_EXTENDED, true, hardware.armDefaultEpsilon),
                new MotorPositionTask(hardware.wrist, CORAL_WRIST_EXTENDED, true, hardware.wristDefaultEpsilon)
            ));
            queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, true, hardware.elevatorDefaultEpsilon));
            queue.AddTask(new CurrentStateTask(STATE_DELIVER, &stateManager));
        }
    }

    if (stateManager.targetState == STATE_DELIVER && mate.GetDPadUpPressed())
    {

    }

    if (stateManager.targetState == STATE_DELIVER && mate.GetDPadDownPressed())
    {
        
    }

    if (mate.GetSquareButtonPressed())
    {
        if (stateManager.targetState == STATE_NEUTRAL)
        {
            queue.AddTask(new TargetStateTask(STATE_FUNNEL, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_FUNNEL, true, hardware.elevatorDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_FUNNEL, true, hardware.wristDefaultEpsilon));
            queue.AddTask(new CurrentStateTask(STATE_FUNNEL, &stateManager));
        }
    }

    if (mate.GetTouchpadButtonPressed())
    {
        if (stateManager.targetState == STATE_NEUTRAL)
        {
            queue.AddTask(new TargetStateTask(STATE_CLIMB, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.flipper, FLIPPER_EXTENDED));
            queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_CLIMB));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_CLIMB));
            queue.AddTask(new CurrentStateTask(STATE_CLIMB, &stateManager));
        }
    }

    queue.Update();
    hardware.Update();
    driver.Update();
    mate.Update();
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

