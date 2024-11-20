#include "util/PID.h"

/**
 * Eval PID
 *
 * @param derivative Make derivative negative to suppress change 
 * 
 */
PID::PID (float proportional, float integral, float derivative)
{
    P = proportional;
    I = integral;
    D = derivative;
}

/**
 * Eval PID
 *
 * @param error Positive error means the target is higher than the current position 
 * 
 */
float PID::update(float error, float dt)
{
    float power = (error * P + (error - lastError) * D + errorAccum * I);

    lastError = error;
    errorAccum += error;

    return power;
}

void PID::reset()
{
    lastError = 0.0;
    errorAccum = 0.0;
}
