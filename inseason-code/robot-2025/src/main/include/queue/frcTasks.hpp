#include "util/MotorController.h"
#include "queue/taskQueue.hpp"

class MotorPositionTask: public Task
{
    MotorController *controller;

    bool wait;
    double epsilon;
    double targetPosition;

    MotorPositionTask(MotorController *ctrlr,double targetPosition,bool wait = false, double epsilon = 0.05);
    void start();
    bool loop();
    void dispose();
};