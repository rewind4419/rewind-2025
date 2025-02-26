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
    frc2::CommandPtr SetHeight(float height);
private:
    ctre::phoenix6::hardware::TalonFX talon1 {ELEVATOR_MOTOR_1_ID};
    ctre::phoenix6::hardware::TalonFX talon2 {ELEVATOR_MOTOR_2_ID};
};
