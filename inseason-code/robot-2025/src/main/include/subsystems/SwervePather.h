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

class SwervePather : public frc2::SubsystemBase
{
public:
    SwervePather(CommandSwerveDrivetrain* drivetrain);

    void Periodic() override;

public:
    // Drive tasks
    frc2::CommandPtr ResetPose(frc::Pose2d r);
    frc2::CommandPtr DriveFor(units::time::second_t timer, units::velocity::meters_per_second_t v);

    frc2::CommandPtr DriveWaypointSimple(frc::Pose2d target);
    
    PID translationPID{0.0, 0.0, 0.0};
    PID rotationPID{0.0, 0.0, 0.0};
public:
    // These could be private with a getter but im just making them public so the driving tasks can access this stuff with a pointer to this class easily
    CommandSwerveDrivetrain* drivetrain;

    swerve::requests::RobotCentric drive_openloop = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);
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
    SwerveCmdDriveWaypointSimple(SwervePather* pather, frc::Pose2d target);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
private:
    SwervePather* m_pather;
    double startTime;
    frc::Pose2d target;
    double lastDistance = 10.0;
};
