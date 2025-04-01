#include "util/MotorController.h"
#include "queue/Queue.h"

class MotorPositionTask : public Task
{
    MotorController *controller;

    double targetPosition;
    bool wait;
    double epsilon;

public:
    MotorPositionTask(MotorController *ctrlr,double targetPosition,bool wait = false, double epsilon = 0.05);
    void Start() override;
    bool Loop() override;
    void End() override;
};

class MotorVelocityTask : public Task
{
    MotorController *controller;

    double targetVelocity;

public:
    MotorVelocityTask(MotorController *ctrlr, double targetVelocity);
    void Start() override;
    bool Loop() override;
    void End() override;
};
