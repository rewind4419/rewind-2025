#include "util/PID.h"

#include "util/maths.h"

// Positive kD resists change, don't do negative kD
PID::PID(float kP, float kI, float kD)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;

    this->lastError = 0.0f;
    this->accumulatedError = 0.0f;
}

// Positive error means positive power needs to be applied to go towards the goal
float PID::step(float error)
{
    this->accumulatedError += error;

    float power = this->kP * (error) + this->kI * (this->accumulatedError) + this->kD * (error - this->lastError);

    this->lastError = error;

    return power;
}

void PID::reset()
{
    this->lastError = 0.0f;
    this->accumulatedError = 0.0f;
}

