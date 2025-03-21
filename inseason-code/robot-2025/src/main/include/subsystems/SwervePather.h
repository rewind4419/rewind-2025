#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>

#include "subsystems/SwerveConstants.h"
#include "subsystems/SwerveDrivetrain.h"

#include "util/PID.h"

#include <units/length.h>
#include <units/time.h>

frc::Pose2d SwapFieldSide(frc::Pose2d x);
frc::Pose2d MirrorLongWays(frc::Pose2d x);
frc::Pose2d MirrorShortWays(frc::Pose2d x);

class SwervePather : public frc2::SubsystemBase
{
public:
    SwervePather(CommandSwerveDrivetrain* drivetrain);

    void Periodic() override;

public:
    // Drive tasks
    frc2::CommandPtr ResetPose(frc::Pose2d r);
    frc2::CommandPtr ResetPoseID(int id);
    frc2::CommandPtr DriveFor(units::time::second_t timer, units::velocity::meters_per_second_t v);

    frc2::CommandPtr DriveWaypointSimple(frc::Pose2d target, 
    units::velocity::meters_per_second_t maxV, units::length::meter_t slop, 
    units::angle::radian_t rSlop, double accelVperTime, double decelVperDistance);

    frc2::CommandPtr LockWheels();

    frc2::CommandPtr DriveBezier();

    frc2::CommandPtr Test();

    frc2::CommandPtr Debug();
    
    PID translationPID {0.0, 0.0, 0.0};
    PID rotationPID {0.0, 0.0, 0.0};
public:
    // These could be private with a getter but im just making them public so the driving tasks can access this stuff with a pointer to this class easily
    CommandSwerveDrivetrain* drivetrain;

    swerve::requests::RobotCentric drive_openloop = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

    swerve::requests::FieldCentric drive_closedloop = swerve::requests::FieldCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::Velocity);

    swerve::requests::SwerveDriveBrake brake {};
};

// Swerve Pather Commands

class SwerveCmdDriveFor : public frc2::CommandHelper<frc2::Command, SwerveCmdDriveFor>
{
public:
    SwerveCmdDriveFor(SwervePather* pather, units::time::second_t timer, units::velocity::meters_per_second_t v);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
private:
    SwervePather* m_pather;
    double startTime;
    units::time::second_t timer;
    units::velocity::meters_per_second_t v;
};

class SwerveCmdDriveWaypointSimple : public frc2::CommandHelper<frc2::Command, SwerveCmdDriveWaypointSimple>
{
public:
    SwerveCmdDriveWaypointSimple(SwervePather* pather, frc::Pose2d target, 
    units::velocity::meters_per_second_t maxV, units::length::meter_t slop, 
    units::angle::radian_t rSlop, double accelVperTime, double decelVperDistance);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
private:
    SwervePather* m_pather;
    double startTime;
    frc::Pose2d target;
    
    units::velocity::meters_per_second_t maxV;
    units::length::meter_t slop;
    units::angle::radian_t rSlop;
    double accelVperTime;
    double decelVperDistance;

    units::length::meter_t lastDistance = 0_m;
    units::angle::radian_t lastRDistance = 0_rad;
};

class SwerveCmdDriveBezier : public frc2::CommandHelper<frc2::Command, SwerveCmdDriveBezier>
{
public:
    SwerveCmdDriveBezier(SwervePather* pather);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
private:
    SwervePather* m_pather;
};
