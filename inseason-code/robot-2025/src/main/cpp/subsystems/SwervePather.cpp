#include "subsystems/SwervePather.h"

SwervePather::SwervePather(CommandSwerveDrivetrain* drivetrain)
{
    this->drivetrain = drivetrain;
}

frc2::CommandPtr DriveFor(units::time::second_t timer, units::velocity::meters_per_second_t v)
{
    // TODO
}