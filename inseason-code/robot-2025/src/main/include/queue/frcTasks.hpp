#include "util/MotorController.h"
#include "queue/taskQueue.hpp"

class MotorPositionTask: public Task
{
    MotorController *controller;

    double targetPosition;
    bool wait;
    double epsilon;

    MotorPositionTask(MotorController *ctrlr,double targetPosition,bool wait = false, double epsilon = 0.05);
    void start();
    bool loop();
    void end();
};