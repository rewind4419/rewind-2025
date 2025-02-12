#include "subsystems/SwervePather.h"

#include <stdio.h>

#include <frc2/command/Commands.h>

SwervePather::SwervePather(CommandSwerveDrivetrain* drivetrain)
{
    this->drivetrain = drivetrain;
}

void SwervePather::Periodic()
{
    // Put logging etc in here
}

frc2::CommandPtr SwervePather::DriveFor(units::time::second_t timer, units::velocity::meters_per_second_t v)
{
    // TODO
    float speed = v.value();

    float timeLength = timer.value();

    float startTime = frc::Timer::GetFPGATimestamp().value();

    return frc2::FunctionalCommand(
        // Reset encoders on command start
        [this, speed] { printf("Starting drive with speed %f\n", speed); },
        // Start driving forward at the start of the command
        [this, startTime, v] {
            this->drivetrain->SetControl(drive_openloop.WithVelocityX(0.0 * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(1_mps) // Drive left with negative X (left)
                .WithRotationalRate(0.0 * MaxAngularRate)); // Drive counterclockwise with negative X (left)
            printf("Elapsed %f seconds\n", frc::Timer::GetFPGATimestamp().value() - startTime); 
        },
        // Stop driving at the end of the command
        [this] (bool interrupted) { printf("Stopping, %s\n", interrupted ? "interrupted" : "not interrupted"); },
        // Return true when done
        [this, startTime, timeLength] { return (frc::Timer::GetFPGATimestamp().value() > startTime + timeLength); },
        // Requires the drive subsystem
        {this->drivetrain}
        // ^ remember this has to be good
    ).ToPtr();  
}
