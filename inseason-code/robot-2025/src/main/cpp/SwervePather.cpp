#include "SwervePather.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "util/maths.h"

// Field length and width from the season field JSON file
frc::Pose2d FIELD_CENTER_POSE { 17.548_m * 0.5, 8.052_m * 0.5, frc::Rotation2d {0_rad}};

frc::Pose2d SwapFieldSide(frc::Pose2d x)
{
    frc::Pose2d flippedRobotPose {FIELD_CENTER_POSE.X() - (x.X() - FIELD_CENTER_POSE.X()), FIELD_CENTER_POSE.Y() - (x.Y() - FIELD_CENTER_POSE.Y()), x.Rotation() + frc::Rotation2d {M_PI * 1_rad}};
    
    return flippedRobotPose;
}

frc::Pose2d MirrorLongWays(frc::Pose2d x)
{
    frc::Pose2d mirrored {x.X(), FIELD_CENTER_POSE.Y() - (x.Y() - FIELD_CENTER_POSE.Y()), frc::Rotation2d{-1.0 * x.Rotation().Radians()}};
    return mirrored;
}

frc::Pose2d MirrorShortWays(frc::Pose2d x)
{
    frc::Pose2d mirrored {FIELD_CENTER_POSE.X() - (x.X() - FIELD_CENTER_POSE.X()), x.Y(), frc::Rotation2d{M_PI * 1_rad - x.Rotation().Radians()}};
    return mirrored;
}

SwervePather::SwervePather(SwerveDrivetrain* d)
{
    this->drivetrain = d;
}

SwerveDriveForTask::SwerveDriveForTask(SwervePather* pather, double durationSeconds, 
double forwardVelocity, double rightVelocity)
{
    this->pather = pather;
    this->durationSeconds = durationSeconds;
    this->velocityForward = forwardVelocity;
    this->velocityRight = rightVelocity;
}

void SwerveDriveForTask::Start()
{
    this->startTime = frc::Timer::GetFPGATimestamp().value();
}

bool SwerveDriveForTask::Loop()
{
    this->pather->drivetrain->SetControl(
        this->pather->drive_openloop.WithVelocityY(0_mps) // Drive forward with negative Y (forward)
        .WithVelocityX(velocityForward * 1_mps) // Drive left with positive X, forward
        .WithVelocityY(-velocityRight * 1_mps)
    );

    return (frc::Timer::GetFPGATimestamp().value() > startTime + this->durationSeconds);
}

void SwerveDriveForTask::End()
{
}

SwerveWaypointTask::SwerveWaypointTask(SwervePather* pather, frc::Pose2d target,
    double maxV, double slop, double rSlop, double accel, double deccel)
{
    this->pather = pather;
    this->target = target;
    this->maxVelocity = maxV;
    this->slop = slop;
    this->rSlop = rSlop;
    this->acceleration = accel;
    this->decceleration = deccel;
}

void SwerveWaypointTask::Start()
{
    printf("Started waypoint\n");
    this->startTime = frc::Timer::GetFPGATimestamp().value();
}

bool SwerveWaypointTask::Loop()
{
    frc::Pose2d currentPose = this->pather->drivetrain->GetState().Pose;

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

    double velocityTarget = clamp(distanceToGoal * decceleration, -(maxVelocity), (maxVelocity));

    // start time in seconds
    double elapsedTime = frc::Timer::GetFPGATimestamp().value() - this->startTime;

    velocityTarget = clamp(velocityTarget, -elapsedTime * acceleration, elapsedTime * acceleration);

    float rotationDistanceToGoal = (targetRot.Radians().value() - currentRot.Radians().value());

    rotationDistanceToGoal = fmod(rotationDistanceToGoal + M_PI, 2 * M_PI) - M_PI;

    // This is not ideal, but modulo wouldn't work with negative numbers without more stuff so idk


    double rotationVelocityTarget = clamp(rotationDistanceToGoal * 9.0f, -2.0f, 2.0f);

    rotationVelocityTarget = clamp(rotationVelocityTarget, -elapsedTime, elapsedTime);

    frc::SmartDashboard::PutNumber("DistanceToTarget", distanceToGoal);
    frc::SmartDashboard::PutNumber("VelocityTarget", velocityTarget);

    frc::SmartDashboard::PutNumber("RotationDistanceToTarget", rotationDistanceToGoal);
    frc::SmartDashboard::PutNumber("RotationVelocityTarget", rotationVelocityTarget);

    this->pather->drivetrain->SetControl(
        this->pather->drive_closedloop
        .WithVelocityX(-(diffTranslation.X().value() / distanceToGoal * velocityTarget) * 1_mps)
        //.WithVelocityX(0.5* 1_mps)
        .WithVelocityY(-(diffTranslation.Y().value() / distanceToGoal * velocityTarget) * 1_mps)
        //.WithVelocityY(0.5 * 1_mps)
        .WithRotationalRate((rotationVelocityTarget) * 1_rad_per_s)
        //.WithRotationalRate(1.0 * 1_rad_per_s)
    );

    frc::SmartDashboard::PutNumber("ApplyingX", (diffTranslation.X().value() / distanceToGoal * velocityTarget));
    frc::SmartDashboard::PutNumber("ApplyingY", (diffTranslation.Y().value() / distanceToGoal * velocityTarget));
    frc::SmartDashboard::PutNumber("ApplyingR", (diffRot.Radians().value() * rotationVelocityTarget));

    double lastDistance = diffTranslation.Distance(frc::Translation2d{0_m, 0_m}).value();
    double lastRDistance = diffRot.Radians().value();
    frc::SmartDashboard::PutNumber("Distance to goal", lastDistance);

    bool done = lastDistance < this->slop && fabsf(lastRDistance) < this->rSlop;
    if(done)
    {
        this->pather->drivetrain->SetControl(
            this->pather->drive_closedloop
            .WithVelocityX(0.0 * 1_mps)
            //.WithVelocityX(0.5* 1_mps)
            .WithVelocityY(0.0 * 1_mps)
            //.WithVelocityY(0.5 * 1_mps)
            .WithRotationalRate(0.0 * 1_rad_per_s)
            //.WithRotationalRate(1.0 * 1_rad_per_s)
        );
    }
    return (lastDistance < this->slop && fabsf(lastRDistance) < this->rSlop);
}

void SwerveWaypointTask::End()
{
    printf("Finished waypoint\n");
}

SwerveLockWheelsTask::SwerveLockWheelsTask(SwervePather* pather)
{
    this->pather = pather;
}

void SwerveLockWheelsTask::Start()
{
    this->pather->drivetrain->SetControl(
        this->pather->brake
    );
}

bool SwerveLockWheelsTask::Loop()
{
    return true;
}

void SwerveLockWheelsTask::End()
{

}
