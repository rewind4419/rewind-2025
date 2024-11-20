#pragma once

class PID
{
public:
    float P;
    float I;
    float D;

    PID (float P, float I, float D);


    float lastError = 0.0;
    float errorAccum = 0.0;

    float update(float error, float dt = 0.02);
    void reset();
};
