#include "util/MotorController.h"
#include "queue/Queue.h"
#include "queue/FrcTasks.h"


MotorPositionTask::MotorPositionTask(MotorController *ctrlr,double targetPosition,bool wait, double epsilon): targetPosition(targetPosition),wait(wait), epsilon(epsilon)
{
    controller = ctrlr;
}
void MotorPositionTask::Start()
{
    controller->SetTargetPosition(targetPosition);
}
bool MotorPositionTask::Loop()
{
    if (wait == false) { return true; }
    double pos = controller->motor.GetPosition().GetValueAsDouble();

    return fabs(pos-targetPosition) < epsilon;
}
void MotorPositionTask::End()
{

}