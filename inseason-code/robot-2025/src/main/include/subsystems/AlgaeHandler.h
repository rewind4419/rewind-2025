/*
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <rev/SparkMax.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Config.h"

class AlgaeHandler : public frc2::SubsystemBase {
public:
    AlgaeHandler();

    void Periodic() override;

    frc2::CommandPtr AlgaeHandler(double speed);
    frc2::CommandPtr AlgaeHandlerTo(float angle);
    frc2::CommandPtr AlgaeHandlerResetPosition();

private:
    ctre::phoenix6::hardware::TalonFX algaeHandlerMotor1 {ALGAE_HANDLER_MOTOR_ID1};
    ctre::phoenix6::hardware::TalonFX algaeHandlerMotor1 {ALGAE_HANDLER_MOTOR_ID2};
};
*/