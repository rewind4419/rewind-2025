#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/length.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Config.h"

class Elevator : public frc2::SubsystemBase
{
public:
    Elevator();

    void Periodic() override;

    // Height is in meters
    frc2::CommandPtr Home();
    frc2::CommandPtr HoldPos();
    frc2::CommandPtr SetHeight(units::angle::turn_t pos);
    frc2::CommandPtr SetHeightProvider(std::function<units::angle::turn_t()> pos);
    frc2::CommandPtr DrivePower(std::function<float()> powerProvider);

    units::angle::turn_t target;

    const float epsilon = 0.15;
private:
    ctre::phoenix6::hardware::TalonFX motor1 {ELEVATOR_MOTOR_1_ID};
    ctre::phoenix6::hardware::TalonFX motor2 {ELEVATOR_MOTOR_2_ID};

    ctre::phoenix6::controls::MotionMagicVoltage elevatorRequest {0_tr};

    ctre::phoenix6::controls::DutyCycleOut elevatorRequestTorque {0};
    
    ctre::phoenix6::controls::Follower elevatorFollower {ELEVATOR_MOTOR_1_ID, true};
};

/*
pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/PWMSparkMax.h>

class ElevatorSubsystem : public frc2::SubsystemBase {
public:
    ElevatorSubsystem();

    void Periodic() override;
    void MoveElevator(double speed);
    void OperateDoor(double position);

private:
    frc::PWMSparkMax m_verticalMotor{0}; // PWM port 0 for vertical movement
    frc::PWMSparkMax m_doorMotor{1};     // PWM port 1 for door operation
};
*/