#include "subsystems/Elevator.h"
#include <stdio.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Subsystem.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;

configs::TalonFXConfiguration abcdtalonFXConfigs{};

Elevator::Elevator() {
    printf("Initialized coral arm\n");

    this->motor1.SetPosition(0.0_tr);

    // in init function
    
    // set slot 0 gains
    auto& slot0Configs = abcdtalonFXConfigs.Slot0;
    // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    // slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 100.0; // A position error of 2.5 rotations results in 12 V output
    // slot0Configs.kI = 0; // no output for integrated error
    // slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    //slot0Configs.kG = 3.0;

    // set Motion Magic settings
    auto& motionMagicConfigs = abcdtalonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100_tps; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 1600_tr_per_s_sq; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 6400_tr_per_s_cu; // Target jerk of 1600 rps/s/s (0.1 seconds)

    //this->coralArmMotor.GetConfigurator().Apply(talonFXConfigs);

    configs::FeedbackConfigs feedback;

    feedback.SensorToMechanismRatio = 1;

    this->motor1.GetConfigurator().Apply(abcdtalonFXConfigs);
    this->motor1.GetConfigurator().Apply(feedback);
}

void Elevator::Periodic()
{
    frc::SmartDashboard::PutNumber("ElevatorHeight", motor1.GetPosition().GetValueAsDouble());
}

frc2::CommandPtr Elevator::Home()
{
    return this->RunOnce([this] {
        this->motor1.SetPosition(0_tr);
    });
}

frc2::CommandPtr Elevator::SetHeight(float height)
{
    return frc2::cmd::Run([this, height] () {
        printf("Elevator height to %f\n", height);

        this->motor1.SetControl(this->elevatorRequest.WithPosition(height * 1_tr));
        
    });
}

/*
ElevatorSubsystem::ElevatorSubsystem() {
    // Initialize motors if necessary
}

void ElevatorSubsystem::Periodic() {
    // Code that runs periodically (e.g., sensor updates)
}

void ElevatorSubsystem::MoveElevator(double speed) {
    m_verticalMotor.Set(speed);
}

void ElevatorSubsystem::OperateDoor(double position) {
    m_doorMotor.Set(position);
}
*/