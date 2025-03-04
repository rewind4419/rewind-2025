#include "subsystems/Elevator.h"
#include <stdio.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Subsystem.h>
#include <frc/smartdashboard/SmartDashboard.h>

Elevator::Elevator()
{
    frc::SmartDashboard::PutNumber("kP", 0.0);
    frc::SmartDashboard::PutNumber("kI", 0.0);
    frc::SmartDashboard::PutNumber("kD", 0.0);
}

void Elevator::Periodic()
{
    ctre::phoenix6::configs::Slot0Configs conf {};

    conf.kP = frc::SmartDashboard::GetNumber("kP", 0.0);
    conf.kI = frc::SmartDashboard::GetNumber("kI", 0.0);
    conf.kD = frc::SmartDashboard::GetNumber("kD", 0.0);
}

frc2::CommandPtr Elevator::Home()
{
    return frc2::FunctionalCommand(
        [this] { },
        [this] {  },
        [this] (bool interrupted) { },
        [this] { return true; },
        {this}
    ).ToPtr();
}

frc2::CommandPtr Elevator::SetHeight(float height)
{

    return frc2::cmd::RunOnce([this, height] () {
        printf("Elevator height to %f\n", height);

        ctre::phoenix6::controls::PositionVoltage request {units::turn_t(height)};

    });
}