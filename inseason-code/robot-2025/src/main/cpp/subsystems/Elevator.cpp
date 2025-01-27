#include "subsystems/Elevator.h"

#include <stdio.h>

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Subsystem.h>

Elevator::Elevator()
{

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

    return frc2::cmd::RunOnce([height] () {printf("going to height %f\n", height);});
}
