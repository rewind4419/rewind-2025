#pragma once

#include "util/MotorController.h"

class Hardware
{
public:
    Hardware();
    ~Hardware();

    void Update();

    MotorController* elevator;
    MotorFollower* elevator2;
    double elevatorDefaultEpsilon = 0.2;

    MotorController* arm;
    double armDefaultEpsilon = 0.04;
    MotorController* wrist;
    double wristDefaultEpsilon = 0.06;
    MotorController* winch;
    MotorController* intake;
    MotorController* pulley;
    MotorController* wheels;
};
