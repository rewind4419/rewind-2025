#include "subsystems/SwervePather.h"

#include <stdio.h>
#include <math.h>

#include "util/maths.h"

#include <units/length.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/smartdashboard/Field2d.h>

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

    frc::SmartDashboard::PutNumber("TeamColorOverride (1 is red, 2 is blue)", 0.0);
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

frc2::CommandPtr SwervePather::ResetPoseID(int id)
{
    
}

frc2::CommandPtr SwervePather::LockWheels()
{
    return this->RunOnce([this] {
        this->drivetrain->SetControl(this->brake);
    });
}

frc2::CommandPtr SwervePather::DriveFor(units::time::second_t timer, units::velocity::meters_per_second_t v)
{
    return SwerveCmdDriveFor(this, timer, v).ToPtr();
}

frc2::CommandPtr SwervePather::DriveWaypointSimple(
    frc::Pose2d target, 
    units::velocity::meters_per_second_t maxV, units::length::meter_t slop, 
    units::angle::radian_t rSlop, double accelVperTime, double decelVperDistance
)
{
    bool flipped = false;

    std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();

    frc::Pose2d potentiallyFlippedTarget = target;

    // double smartDashboardOverride = frc::SmartDashboard::GetNumber("TeamColorOverride (1 is red, 2 is blue)", 0.0);

    // if (smartDashboardOverride > 0.1)
    // {
    //     printf("Using smartdashboard override: %f\n", smartDashboardOverride);
    //     if (smartDashboardOverride > 1.5) {
    //         printf("Smartdashboard override is 2, forcing blue\n");
    //         potentiallyFlippedTarget = SwapFieldSide(target);
    //     }
    //     else
    //     {
    //         printf("Smartdashboard override is 1, keeping red\n");
    //     }
    // }
    // else
    {
        if (!alliance.has_value()) {printf("Bruh the optional returned no field color, defaulting to RED. :(\n");}

        if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kBlue)
        {
            printf("Flipping field side for blue! :)\n");
            potentiallyFlippedTarget = SwapFieldSide(target);
        }

        if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed)
        {
            printf("Keeping red side! :)\n");
        }
    }
    
    return SwerveCmdDriveWaypointSimple(this, potentiallyFlippedTarget, maxV, slop, rSlop, accelVperTime, decelVperDistance).ToPtr();

    //return SwerveCmdDriveWaypointSimple(this, target, maxV, slop, rSlop, accelVperTime, decelVperDistance).ToPtr();
}

frc2::CommandPtr SwervePather::DriveBezier()
{
    return SwerveCmdDriveBezier(this).ToPtr();
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

    this->m_pather->drivetrain->SetControl(
        this->m_pather->drive_openloop.WithVelocityY(0_mps) // Drive forward with negative Y (forward)
        .WithVelocityX(v) // Drive left with positive X, forward
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

SwerveCmdDriveWaypointSimple::SwerveCmdDriveWaypointSimple(
    SwervePather* pather, frc::Pose2d target, 
    units::velocity::meters_per_second_t maxV, units::length::meter_t slop, 
    units::angle::radian_t rSlop, double accelVperTime, double decelVperDistance
)
{
    AddRequirements(pather->drivetrain);
    this->m_pather = pather;
    this->target = target;
    
    this->maxV = maxV;
    this->slop = slop;
    this->rSlop = rSlop;
    this->accelVperTime = accelVperTime;
    this->decelVperDistance = decelVperDistance;
}

void SwerveCmdDriveWaypointSimple::Initialize()
{
    printf("Started waypoint\n");
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

    double velocityTarget = clamp(distanceToGoal * decelVperDistance, -(maxV.value()), (maxV.value()));

    // start time in seconds
    double elapsedTime = frc::Timer::GetFPGATimestamp().value() - this->startTime;

    velocityTarget = clamp(velocityTarget, -elapsedTime * accelVperTime, elapsedTime * accelVperTime);

    float rotationDistanceToGoal = abs(targetRot.Radians().value() - currentRot.Radians().value());

    double rotationVelocityTarget = clamp(rotationDistanceToGoal * 12.0f, -2.0f, 2.0f);

    rotationVelocityTarget = clamp(rotationVelocityTarget, -elapsedTime, elapsedTime);

    frc::SmartDashboard::PutNumber("DistanceToTarget", distanceToGoal);
    frc::SmartDashboard::PutNumber("VelocityTarget", velocityTarget);

    frc::SmartDashboard::PutNumber("RotationDistanceToTarget", rotationDistanceToGoal);
    frc::SmartDashboard::PutNumber("RotationVelocityTarget", rotationVelocityTarget);

    this->m_pather->drivetrain->SetControl(
        this->m_pather->drive_closedloop
        .WithVelocityX(-(diffTranslation.X().value() / distanceToGoal * velocityTarget) * 1_mps)
        //.WithVelocityX(0.5* 1_mps)
        .WithVelocityY(-(diffTranslation.Y().value() / distanceToGoal * velocityTarget) * 1_mps)
        //.WithVelocityY(0.5 * 1_mps)
        .WithRotationalRate((diffRot.Radians().value() * rotationVelocityTarget) * 1_rad_per_s)
        //.WithRotationalRate(1.0 * 1_rad_per_s)
    );

    frc::SmartDashboard::PutNumber("ApplyingX", (diffTranslation.X().value() / distanceToGoal * velocityTarget));
    frc::SmartDashboard::PutNumber("ApplyingY", (diffTranslation.Y().value() / distanceToGoal * velocityTarget));
    frc::SmartDashboard::PutNumber("ApplyingR", (diffRot.Radians().value() * rotationVelocityTarget));

    this->lastDistance = diffTranslation.Distance(frc::Translation2d{0_m, 0_m});
    this->lastRDistance = diffRot.Radians();
    frc::SmartDashboard::PutNumber("Distance to goal", this->lastDistance.value());

    // printf("Distance: %f\n", this->lastDistance);
}

void SwerveCmdDriveWaypointSimple::End(bool interrupted)
{
    printf("Finished waypoint\n");
    frc::SmartDashboard::PutString("Status", "Finished waypoint");
}

bool SwerveCmdDriveWaypointSimple::IsFinished()
{
    return (this->lastDistance < this->slop && fabsf(this->lastRDistance.value()) < this->rSlop.value());
}

#include "swervepather/bezier.inc"

BezierPath path {Vec3 {0.0, 0.0, 0.0}, Vec3{2.0, 0.0, 0.0}, Vec3{2.0, -2.0, 0.0}};

SwerveCmdDriveBezier::SwerveCmdDriveBezier(SwervePather* pather)
{
    AddRequirements(pather->drivetrain);
    this->m_pather = pather;

    
}

void SwerveCmdDriveBezier::Initialize()
{
    // printf("Starting bezier\n");


}

void SwerveCmdDriveBezier::Execute()
{
    frc::Pose2d robotPose = this->m_pather->drivetrain->GetState().Pose;
    frc::ChassisSpeeds robotVelocity = this->m_pather->drivetrain->GetState().Speeds;

    frc::SmartDashboard::PutNumber("Bezier - Input Pose X", robotPose.X().value());
    frc::SmartDashboard::PutNumber("Bezier - Input Pose Y", robotPose.Y().value());
    frc::SmartDashboard::PutNumber("Bezier - Input Pose R", robotPose.Rotation().Radians().value());

    frc::SmartDashboard::PutNumber("Bezier - Input Velocity X", robotVelocity.vx.value());
    frc::SmartDashboard::PutNumber("Bezier - Input Velocity Y", robotVelocity.vy.value());
    frc::SmartDashboard::PutNumber("Bezier - Input Velocity R", robotVelocity.omega.value());

    Vec3 robotTargetVelocity = path.getRobotControlVelocity(
        Vec3{robotPose.X().value(), robotPose.Y().value(), robotPose.Rotation().Radians().value()},
        Vec3{robotVelocity.vx.value(), robotVelocity.vy.value(), robotVelocity.omega.value()},
        0.02
    );

    double targetVelocityMagnitude = clamp(robotTargetVelocity.mag(), -1.0, 1.0);

    Vec3 clampedRobotTargetVelocity = robotTargetVelocity.normalized() * targetVelocityMagnitude;

    this->m_pather->drivetrain->SetControl(
        this->m_pather->drive_closedloop
        .WithVelocityX(clampedRobotTargetVelocity.x * 1_mps)
        .WithVelocityY(clampedRobotTargetVelocity.y * 1_mps)
        .WithRotationalRate(clampedRobotTargetVelocity.r * 1_rad_per_s)
    );

    frc::SmartDashboard::PutNumber("Bezier - Target Velocity X", robotTargetVelocity.x);
    frc::SmartDashboard::PutNumber("Bezier - Target Velocity Y", robotTargetVelocity.y);
    frc::SmartDashboard::PutNumber("Bezier - Target Velocity R", robotTargetVelocity.r);
    frc::SmartDashboard::PutNumber("Bezier - Target Velocity X clamped", clampedRobotTargetVelocity.x);
    frc::SmartDashboard::PutNumber("Bezier - Target Velocity Y clamped", clampedRobotTargetVelocity.y);
    frc::SmartDashboard::PutNumber("Bezier - Target Velocity R clamped", clampedRobotTargetVelocity.r);
}

void SwerveCmdDriveBezier::End(bool interrupted)
{
    frc::SmartDashboard::PutString("Status", "Finished waypoint");
}

bool SwerveCmdDriveBezier::IsFinished()
{
    return false;
}
