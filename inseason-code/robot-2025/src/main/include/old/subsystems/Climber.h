/*
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <rev/SparkMax.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Config.h"

class Climber : public frc2::SubsystemBase {
public:
    Climber();
    void Periodic() override;
    frc2::CommandPtr Climber(double speed);
    
    frc2::CommandPtr ClimberResetPosition();
   
private:
    ctre::phoenix6::hardware::TalonFX climberMotor {WINCH_MOTOR_ID, "rio"};
    //frc::DCMotor br = frc::DCMotor::KrakenX60();
}
*/