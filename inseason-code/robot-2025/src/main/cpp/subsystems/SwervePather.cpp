#include "subsystems/SwervePather.h"

#include <stdio.h>
#include <math.h>

#include "util/maths.h"

#include <units/length.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

SwervePather::SwervePather(CommandSwerveDrivetrain* drivetrain)
{
    this->drivetrain = drivetrain;

    // // SmartDashboard PID Tuning
    // frc::SmartDashboard::PutNumber("Translation kP", 0.0);
    // frc::SmartDashboard::PutNumber("Translation kI", 0.0);
    // frc::SmartDashboard::PutNumber("Translation kD", 0.0);

    // frc::SmartDashboard::PutNumber("Rotation kP", 0.0);
    // frc::SmartDashboard::PutNumber("Rotation kI", 0.0);
    // frc::SmartDashboard::PutNumber("Rotation kD", 0.0);

    // frc::SmartDashboard::PutNumber("Manual X Target", 0.0);
    // frc::SmartDashboard::PutNumber("Manual Y Target", 0.0);
    // frc::SmartDashboard::PutNumber("Manual R Target", 0.0);
}

void SwervePather::Periodic()
{
    // Put logging etc in here
    // this->translationPID.kP = frc::SmartDashboard::GetNumber("Translation kP", 0.0);
    // this->translationPID.kI = frc::SmartDashboard::GetNumber("Translation kI", 0.0);
    // this->translationPID.kD = frc::SmartDashboard::GetNumber("Translation kD", 0.0);

    // this->rotationPID.kP = frc::SmartDashboard::GetNumber("Rotation kP", 0.0);
    // this->rotationPID.kI = frc::SmartDashboard::GetNumber("Rotation kI", 0.0);
    // this->rotationPID.kD = frc::SmartDashboard::GetNumber("Rotation kD", 0.0);

    // printf("PIDS: %f, %f, %f - %f, %f, %f\n", 
    //     this->translationPID.kP,
    //     this->translationPID.kI,
    //     this->translationPID.kD,

    //     this->rotationPID.kP,
    //     this->rotationPID.kI,
    //     this->rotationPID.kD
    // ); 
}

frc2::CommandPtr SwervePather::Debug()
{
    return this->Run([this] {
        this->drivetrain->SetControl(
            this->drive_closedloop
            .WithVelocityX(frc::SmartDashboard::GetNumber("Manual X Target", 0.0) * 1_mps)
            .WithVelocityY(frc::SmartDashboard::GetNumber("Manual Y Target", 0.0) * 1_mps)
            .WithRotationalRate(frc::SmartDashboard::GetNumber("Manual R Target", 0.0) * 1_rad_per_s)
        );
    });
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

frc2::CommandPtr SwervePather::DriveWaypointSimple(frc::Pose2d target, units::velocity::meters_per_second_t maxV, double slopDistance)
{
    return SwerveCmdDriveWaypointSimple(this, target, maxV, slopDistance).ToPtr();
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
    // frc::SmartDashboard::PutString("Status", "Starting periodic drive");
    this->startTime = frc::Timer::GetFPGATimestamp().value();
}

void SwerveCmdDriveFor::Execute()
{
    units::angular_velocity::radians_per_second_t rVel {this->v.value()};

    this->m_pather->drivetrain->SetControl(
        this->m_pather->drive_closedloop.WithVelocityY(0_mps) // Drive forward with negative Y (forward)
        .WithVelocityX(0_mps) // Drive left with positive X, forward

        .WithRotationalRate(rVel)
    ); // Drive counterclockwise with negative X (left)
    
    //printf("Elapsed %f seconds\n", frc::Timer::GetFPGATimestamp().value() - startTime);
}

void SwerveCmdDriveFor::End(bool interrupted)
{
    // frc::SmartDashboard::PutString("Status", "Finished periodic drive");
    //printf("Stopping, %s\n", interrupted ? "interrupted" : "not interrupted");
}

bool SwerveCmdDriveFor::IsFinished()
{
    return (frc::Timer::GetFPGATimestamp().value() > startTime + this->timer.value());
}

SwerveCmdDriveWaypointSimple::SwerveCmdDriveWaypointSimple(SwervePather* pather, frc::Pose2d target, units::velocity::meters_per_second_t maxV, double slopDistance)
{
    AddRequirements(pather->drivetrain);
    this->m_pather = pather;
    this->maxV = maxV;
    this->target = target;
    this->slopDistance = slopDistance;
}

void SwerveCmdDriveWaypointSimple::Initialize()
{
    frc::SmartDashboard::PutString("Status", "Started waypoint");

    this->startTime = frc::Timer::GetFPGATimestamp().value();
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
    frc::SmartDashboard::PutNumber("DiffR", diffRot.Radians().value());

    float distanceToGoal = sqrtf(diffTranslation.X().value() * diffTranslation.X().value() + diffTranslation.Y().value() * diffTranslation.Y().value());

    double velocityTarget = clamp(distanceToGoal * 3.0f, -1.0f, 1.0f);

    // start time in seconds
    double elapsedTime = frc::Timer::GetFPGATimestamp().value() - this->startTime;
    elapsedTime *= 2.0;
    velocityTarget = clamp(velocityTarget, -elapsedTime, elapsedTime);

    float rotationDistanceToGoal = abs(targetRot.Radians().value() - currentRot.Radians().value());

    double rotationVelocityTarget = clamp(rotationDistanceToGoal * 10.0f, -2.0f, 2.0f);

    rotationVelocityTarget = clamp(rotationVelocityTarget, -elapsedTime, elapsedTime);

    frc::SmartDashboard::PutNumber("DistanceToTarget", distanceToGoal);
    frc::SmartDashboard::PutNumber("VelocityTarget", velocityTarget);

    frc::SmartDashboard::PutNumber("RotationDistanceToTarget", rotationDistanceToGoal);
    frc::SmartDashboard::PutNumber("RotationVelocityTarget", rotationVelocityTarget);

    this->m_pather->drivetrain->SetControl(
        this->m_pather->drive_closedloop
        .WithVelocityX((diffTranslation.X().value() / distanceToGoal * velocityTarget) * 1_mps)
        .WithVelocityY((diffTranslation.Y().value() / distanceToGoal * velocityTarget) * 1_mps)
        .WithRotationalRate((diffRot.Radians().value() * rotationVelocityTarget) * 1_rad_per_s)
    );

    frc::SmartDashboard::PutNumber("ApplyingX", (diffTranslation.X().value() / distanceToGoal * velocityTarget));
    frc::SmartDashboard::PutNumber("ApplyingY", (diffTranslation.Y().value() / distanceToGoal * velocityTarget));
    frc::SmartDashboard::PutNumber("ApplyingR", (diffRot.Radians().value() * rotationVelocityTarget));

    this->lastDistance = diffTranslation.Distance(frc::Translation2d{0_m, 0_m}).value();
    this->lastRotDistance = diffRot.Radians().value();
    frc::SmartDashboard::PutNumber("Distance to goal", this->lastDistance);

    // printf("Distance: %f\n", this->lastDistance);
}

void SwerveCmdDriveWaypointSimple::End(bool interrupted)
{
    frc::SmartDashboard::PutString("Status", "Finished waypoint");
}

bool SwerveCmdDriveWaypointSimple::IsFinished()
{
    return (this->lastDistance < slopDistance && this->lastRotDistance < 0.15f);
}
