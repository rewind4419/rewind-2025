#include "util/MotorController.h"
#include "queue/taskQueue.hpp"
#include "queue/frcTasks.hpp"


MotorPositionTask::MotorPositionTask(MotorController *ctrlr,double targetPosition,bool wait = false, double epsilon = 0.05): targetPosition(targetPosition),wait(wait), epsilon(epsilon)
{
    controller = ctrlr;
}
void MotorPositionTask::start()
{
    controller->setTargetPosition(targetPosition);
}
bool MotorPositionTask::loop()
{
    double pos = controller->motor.GetPosition().GetValueAsDouble();

    return fabs(pos-targetPosition) < epsilon;
}
void MotorPositionTask::dispose()
{
    delete this;
}