#pragma once

class PID
{
public:
    PID(float kP, float kI, float kD);

    float step(float error);

    void reset();

    float lastError = 0.0;
    float accumulatedError = 0.0;

    float kP = 0.0;
    float kI = 0.0;
    float kD = 0.0;
};
