#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>

#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

float clamp(float x, float min, float max);
float len(float x, float y);
float lenSq(float x, float y);

int clamp(int x, int min, int max);
double clamp(double x, double min, double max);

units::angle::turn_t clamp(units::angle::turn_t x, units::angle::turn_t min, units::angle::turn_t max);
units::velocity::meters_per_second_t clamp(units::velocity::meters_per_second_t x, units::velocity::meters_per_second_t min, units::velocity::meters_per_second_t max);units::velocity::meters_per_second_t clamp(units::velocity::meters_per_second_t x, units::velocity::meters_per_second_t min, units::velocity::meters_per_second_t max);
units::angular_velocity::radians_per_second_t clamp(units::angular_velocity::radians_per_second_t x, units::angular_velocity::radians_per_second_t min, units::angular_velocity::radians_per_second_t max);
// General purpose vector 2 class
class V2
{
public:
    float x;
    float y;

    V2() {this->x = 0.0f; this->y = 0.0f;}
    V2(float x, float y) {this->x = x; this->y = y;}

    // These vector 2s can be added with the plus symbol
    V2 operator+(V2& r)
    {
        return V2{this->x + r.x, this->y + r.y};
    }

    V2 operator-(V2& r)
    {
        return V2{this->x - r.x, this->y - r.y};
    }
    
    float dot(V2& r)
    {
        return this->x * r.x + this->y * r.y;
    }

    float cross(V2& r)
    {
        return this->x * r.y - this->y * r.x;
    }

    float length()
    {
        return sqrtf(this->x * this->x + this->y * this->y);
    }

    // Can be used instead of length in some cases, and its faster because it avoids a slow square root calculation
    float lengthSquared()
    {
        return this->x * this->x + this->y * this->y;
    }

    V2 normalize()
    {
        float len = this->length();
        return V2 {this->x / len, this->y / len};
    }
};

// X and Y are in meters following the Pose2D convention. Positive rotation is counterclockwise, and in radians.
class RPose
{
public:
    V2 translation;
    float r;

    RPose(V2 translation, float r)
    {
        this->translation = translation;
        this->r = r;
    }
};

RPose RPosefromFRCPose(frc::Pose2d pose);

frc::Pose2d RPosetoFrcPose(RPose pose);
