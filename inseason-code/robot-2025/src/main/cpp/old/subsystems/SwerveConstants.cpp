#include "subsystems/SwerveConstants.h"
#include "subsystems/SwerveDrivetrain.h"

CommandSwerveDrivetrain TunerConstants::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
