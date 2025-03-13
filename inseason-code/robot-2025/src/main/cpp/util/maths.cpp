#include "util/maths.h"

float clamp(float x, float min, float max)
{
    if (x < min) {return min;}
    if (x > max) {return max;}
    return x;
}

float len(float x, float y)
{
    return sqrtf(x*x+y*y);
}

float lenSq(float x, float y)
{
    return(x*x+y*y);
}

int clamp(int x, int min, int max)
{
    if (x < min) {return min;}
    if (x > max) {return max;}
    return x;
}

units::angle::turn_t clamp(units::angle::turn_t x, units::angle::turn_t min, units::angle::turn_t max)
{
    if (x < min) {return min;}
    if (x > max) {return max;}
    return x;
}

RPose RPosefromFRCPose(frc::Pose2d pose)
{
    return RPose {
        V2 {
            pose.X().value(),
            pose.Y().value()
        },
        pose.Rotation().Radians().value()
    };
}

frc::Pose2d RPosetoFrcPose(RPose pose)
{
    //Todo
    //return frc::Pose2d {(pose.translation.x)_m, (pose.translation.y)_m, frc::Rotation2d{} }
}
