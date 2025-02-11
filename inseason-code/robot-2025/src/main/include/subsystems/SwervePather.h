#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
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
};
