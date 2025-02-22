#include "subsystems/SwervePather.h"

#include <stdio.h>
#include <math.h>

#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

SwervePather::SwervePather(CommandSwerveDrivetrain* drivetrain)
{
    this->drivetrain = drivetrain;

    // SmartDashboard PID Tuning
    frc::SmartDashboard::PutNumber("Translation kP", 0.0);
    frc::SmartDashboard::PutNumber("Translation kI", 0.0);
    frc::SmartDashboard::PutNumber("Translation kD", 0.0);

    frc::SmartDashboard::PutNumber("Rotation kP", 0.0);
    frc::SmartDashboard::PutNumber("Rotation kI", 0.0);
    frc::SmartDashboard::PutNumber("Rotation kD", 0.0);
}

void SwervePather::Periodic()
{
    // Put logging etc in here
    this->translationPID.kP = frc::SmartDashboard::GetNumber("Translation kP", 0.0);
    this->translationPID.kI = frc::SmartDashboard::GetNumber("Translation kI", 0.0);
    this->translationPID.kD = frc::SmartDashboard::GetNumber("Translation kD", 0.0);

    this->rotationPID.kP = frc::SmartDashboard::GetNumber("Rotation kP", 0.0);
    this->rotationPID.kI = frc::SmartDashboard::GetNumber("Rotation kI", 0.0);
    this->rotationPID.kD = frc::SmartDashboard::GetNumber("Rotation kD", 0.0);

    // printf("PIDS: %f, %f, %f - %f, %f, %f\n", 
    //     this->translationPID.kP,
    //     this->translationPID.kI,
    //     this->translationPID.kD,

    //     this->rotationPID.kP,
    //     this->rotationPID.kI,
    //     this->rotationPID.kD
    // ); 
}

frc2::CommandPtr SwervePather::ResetPose(frc::Pose2d r)
{
    return this->RunOnce([this, r] {
        this->drivetrain->ResetPose(r);
    });
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
    frc::SmartDashboard::PutString("Status", "Starting periodic drive");
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
    frc::SmartDashboard::PutString("Status", "Finished periodic drive");
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
    frc::SmartDashboard::PutString("Status", "Started waypoint");
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

    frc::SmartDashboard::PutNumber("DiffX", diffTranslation.X().value());
    frc::SmartDashboard::PutNumber("DiffY", diffTranslation.Y().value());

    float diffMagnitude = sqrtf(diffTranslation.X().value() * diffTranslation.X().value() + diffTranslation.Y().value() * diffTranslation.Y().value());

    float factor = clamp(diffMagnitude, -0.5, 0.5) / diffMagnitude;

    this->m_pather->drivetrain->SetControl(
        this->m_pather->drive_openloop
        .WithVelocityY((diffTranslation.Y().value() * factor) * 1_mps)
        .WithVelocityX((diffTranslation.X().value() * factor) * 1_mps)
        .WithRotationalRate(0.0_rad_per_s)
    );

    this->lastDistance = diffTranslation.Distance(frc::Translation2d{0_m, 0_m}).value();
    frc::SmartDashboard::PutNumber("Distance to goal", this->lastDistance);

    // printf("Distance: %f\n", this->lastDistance);
}

void SwerveCmdDriveWaypointSimple::End(bool interrupted)
{
    frc::SmartDashboard::PutString("Status", "Finished waypoint");
}

bool SwerveCmdDriveWaypointSimple::IsFinished()
{
   // return false;
    return (this->lastDistance < 0.35);
}
