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
};
