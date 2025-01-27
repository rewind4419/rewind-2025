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

    talon.GetConfigurator().Apply(conf);

    frc::SmartDashboard::PutNumber("Current", talon.GetSupplyCurrent().GetValueAsDouble());
}

frc2::CommandPtr Elevator::SetHeight(float height)
{
    // frc2::FunctionalCommand(
    //     // Reset encoders on command start
    //     [this] { m_drive.ResetEncoders(); },
    //     // Start driving forward at the start of the command
    //     [this] { m_drive.ArcadeDrive(ac::kAutoDriveSpeed, 0); },
    //     // Stop driving at the end of the command
    //     [this] (bool interrupted) { m_drive.ArcadeDrive(0, 0); },
    //     // End the command when the robot's driven distance exceeds the desired value
    //     [this] { return true; },
    //     // Requires the drive subsystem
    //     {this}
    // ).ToPtr();

    return frc2::cmd::RunOnce([this, height] () {
        // this->m_power = height;
        // this->talon.Set(height);

        ctre::phoenix6::controls::PositionVoltage request {units::turn_t(height)};

        this->talon.SetControl(request.WithSlot(0));
    });
}
