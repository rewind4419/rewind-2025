#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/length.h>

#include <ctre/phoenix6/TalonFX.hpp>

class Elevator : public frc2::SubsystemBase
{
public:
    Elevator();

    void Periodic() override;

    // Height is in meters
    frc2::CommandPtr SetHeight(float height);
private:
    // ctre::phoenix6::hardware::TalonFX talon {2};
};
