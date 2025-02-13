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
    return SwerveCmdDriveFor(this, timer, v).ToPtr();
}

frc2::CommandPtr SwervePather::DriveWaypointSimple(frc::Pose2d target)
{
    return SwerveCmdDriveWaypointSimple(this, target).ToPtr();
}

SwerveCmdDriveFor::SwerveCmdDriveFor(SwervePather* pather, units::time::second_t timer, units::velocity::meters_per_second_t v)
{
    AddRequirements(pather->drivetrain);
    this->m_pather = pather;
    this->timer = timer;
    this->v = v;
}

void SwerveCmdDriveFor::Initialize()
{
    //printf("Starting drive with speed %f\n", this->v.value());
    this->startTime = frc::Timer::GetFPGATimestamp().value();
}

void SwerveCmdDriveFor::Execute()
{
    this->m_pather->drivetrain->SetControl(
        this->m_pather->drive_openloop.WithVelocityY(0_mps) // Drive forward with negative Y (forward)
        .WithVelocityX(this->v) // Drive left with positive X, forward
        .WithRotationalRate(0.0_rad_per_s)
    ); // Drive counterclockwise with negative X (left)
    
    //printf("Elapsed %f seconds\n", frc::Timer::GetFPGATimestamp().value() - startTime);
}

void SwerveCmdDriveFor::End(bool interrupted)
{
    //printf("Stopping, %s\n", interrupted ? "interrupted" : "not interrupted");
}

bool SwerveCmdDriveFor::IsFinished()
{
    return (frc::Timer::GetFPGATimestamp().value() > startTime + this->timer.value());
}

SwerveCmdDriveWaypointSimple::SwerveCmdDriveWaypointSimple(SwervePather* pather, frc::Pose2d target)
{
    AddRequirements(pather->drivetrain);
    this->m_pather = pather;
    this->target = target;
}

void SwerveCmdDriveWaypointSimple::Initialize()
{

}

void SwerveCmdDriveWaypointSimple::Execute()
{
    frc::Pose2d currentPose = this->m_pather->drivetrain->GetState().Pose;

    frc::Translation2d currentTranslation = currentPose.Translation();
    frc::Translation2d targetTranslation = this->target.Translation();

    frc::Rotation2d currentRot = currentPose.Rotation();
    frc::Rotation2d targetRot = this->target.Rotation();

    frc::Translation2d diffTranslation = targetTranslation - currentTranslation;
    frc::Rotation2d diffRot = targetRot - currentRot;

    printf("DiffX: %f, Diff: %f\n", diffTranslation.X().value(), diffTranslation.Y().value());

    this->m_pather->drivetrain->SetControl(
        this->m_pather->drive_openloop
        .WithVelocityY((diffTranslation.Y().value() * 0.1) * 1_mps)
        .WithVelocityX((diffTranslation.X().value() * 0.1) * 1_mps)
        .WithRotationalRate(0.0_rad_per_s)
    );

    this->lastDistance = diffTranslation.Distance(frc::Translation2d{0_m, 0_m}).value();

    printf("Distance: %f\n", this->lastDistance);
}

void SwerveCmdDriveWaypointSimple::End(bool interrupted)
{

}

bool SwerveCmdDriveWaypointSimple::IsFinished()
{
    return (this->lastDistance < 0.1);
}
