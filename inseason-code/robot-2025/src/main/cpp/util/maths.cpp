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

double clamp(double x, double min, double max)
{
    if (x < min) {return min;}
    if (x > max) {return max;}
    return x;
}

double deadzone(double x, double deadzoneMax)
{
  if (fabsf(x) < deadzoneMax)
  {
    return 0.0;
  }

  return clamp(fabsf(x) * (1 + deadzoneMax) - deadzoneMax, 0.0, 1.0) * (x >= 0 ? 1.0 : -1.0);
}

units::angle::turn_t clamp(units::angle::turn_t x, units::angle::turn_t min, units::angle::turn_t max)
{
    if (x < min) {return min;}
    if (x > max) {return max;}
    return x;
}

units::velocity::meters_per_second_t clamp(units::velocity::meters_per_second_t x, units::velocity::meters_per_second_t min, units::velocity::meters_per_second_t max)
{
    if (x < min) {return min;}
    if (x > max) {return max;}
    return x;
}

units::angular_velocity::radians_per_second_t clamp(units::angular_velocity::radians_per_second_t x, units::angular_velocity::radians_per_second_t min, units::angular_velocity::radians_per_second_t max)
{
    if (x < min) {return min;}
    if (x > max) {return max;}
    return x;
}

RPose RPosefromFRCPose(frc::Pose2d pose)
{
    return RPose {
        V2 {
            (float)pose.X().value(),
            (float)pose.Y().value()
        },
        (float)pose.Rotation().Radians().value()
    };
}


