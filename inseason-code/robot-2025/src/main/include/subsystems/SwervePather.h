#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "subsystems/SwerveConstants.h"
#include "subsystems/SwerveDrivetrain.h"

#include <units/length.h>
#include <units/time.h>

class SwervePather : public frc2::SubsystemBase
{
public:
    SwervePather(CommandSwerveDrivetrain* drivetrain);

    void Periodic() override;

    // Test function

    // Height is in meters
    frc2::CommandPtr DriveFor(units::time::second_t timer, units::velocity::meters_per_second_t v);
private:
    CommandSwerveDrivetrain* drivetrain;

    units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity


    swerve::requests::RobotCentric drive_openloop = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.05).WithRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    
};
