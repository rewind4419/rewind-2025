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

frc2::CommandPtr SwervePather::ResetPoseID(int id)
{
    
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
    return SwerveCmdDriveWaypointSimple(this, target, maxV, slop, rSlop, accelVperTime, decelVperDistance).ToPtr();
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

    double rotationVelocityTarget = clamp(rotationDistanceToGoal * 10.0f, -2.0f, 2.0f);

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
