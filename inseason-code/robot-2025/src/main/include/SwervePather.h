#pragma once

#include "SwerveConstants.h"
#include "SwerveDrivetrain.h"

#include "queue/Queue.h"

#include <frc/geometry/Pose2d.h>

frc::Pose2d SwapFieldSide(frc::Pose2d x);
frc::Pose2d MirrorLongWays(frc::Pose2d x);
frc::Pose2d MirrorShortWays(frc::Pose2d x);

class SwervePather
{
public:
    SwervePather(SwerveDrivetrain* d);

    // These could be private with a getter but im just making them public so the driving tasks can access this stuff with a pointer to this class easily
    SwerveDrivetrain* drivetrain;

    swerve::requests::RobotCentric drive_openloop = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

    swerve::requests::FieldCentric drive_closedloop = swerve::requests::FieldCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::Velocity);

    swerve::requests::SwerveDriveBrake brake {};
};

class SwerveDriveForTask : public Task
{
    SwervePather* pather;
    double startTime;
    double durationSeconds;
    double velocityForward;
    double velocityRight;
public:
    // Robot centric and dead reckoning
    SwerveDriveForTask(SwervePather* pather, double durationSeconds, double forwardVelocity, double rightVelocity);

    void Start() override;
    bool Loop() override;
    void End() override;
};

class SwerveWaypointTask : public Task
{
    SwervePather* pather;
    double startTime;
    frc::Pose2d target;
    double maxVelocity;
    double slop;
    double rSlop;
    double acceleration; // M/S per second
    double decceleration; // M/S drop per meter
public:
    SwerveWaypointTask(SwervePather* pather, frc::Pose2d target,
    double maxV, double slop, double rSlop, double accel, double deccel);

    void Start() override;
    bool Loop() override;
    void End() override;
};

class SwerveLockWheelsTask : public Task
{
    SwervePather* pather;
public:
    SwerveLockWheelsTask(SwervePather* pather);

    void Start() override;
    bool Loop() override;
    void End() override;
};
