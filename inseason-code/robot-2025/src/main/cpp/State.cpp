#include "State.h"

#include "util/maths.h"
#include "queue/FrcTasks.h"
#include "Config.h"

DeliverHeight StateManager::GetNextHeight()
{
    return (DeliverHeight)clamp(this->height + 1, HEIGHT_ZERO, HEIGHT_L4);
}

DeliverHeight StateManager::GetPreviousHeight()
{
    return (DeliverHeight)clamp(this->height - 1, HEIGHT_ZERO, HEIGHT_L4);
}

void StateManager::GoToDeliverHeight(Queue* queue, MotorController* elevator, MotorController* wrist, MotorController* arm, DeliverHeight newHeight)
{
    if (newHeight == this->height) {return;}

    switch (this->height)
    {
    case HEIGHT_L4:
        queue->AddTask(new MotorPositionTask(arm, CORAL_ARM_TRANSIT, true, 0.025));
        queue->AddTask(new MotorPositionTask(elevator, ELEVATOR_DELIVER_MID, true, 0.25));
        break;
    default:
        break;
    };

    switch (newHeight)
    {
    case HEIGHT_ZERO:
        queue->AddTask(new MotorPositionTask(elevator, ELEVATOR_MIN, false, 0.25));
        queue->AddTask(new MotorPositionTask(wrist, CORAL_WRIST_EXTENDED));
        break;
    case HEIGHT_L2:
        queue->AddTask(new MotorPositionTask(elevator, ELEVATOR_DELIVER_LOW, false, 0.25));
        queue->AddTask(new MotorPositionTask(wrist, CORAL_WRIST_EXTENDED));
        break;
    case HEIGHT_L3:
        queue->AddTask(new MotorPositionTask(elevator, ELEVATOR_DELIVER_MID, false, 0.25));
        queue->AddTask(new MotorPositionTask(wrist, CORAL_WRIST_EXTENDED));
        break;
    case HEIGHT_L4:
        // queue->AddTask(new MotorPositionTask(elevator, ELEVATOR_DELIVER_MID, true, 0.25));
        // queue->AddTask(new MotorPositionTask(arm, CORAL_ARM_TRANSIT, true, 0.025));
        queue->AddTask(new MotorPositionTask(elevator, ELEVATOR_DELIVER_HIGH, false, 0.25));
        queue->AddTask(new MotorPositionTask(wrist, CORAL_WRIST_EXTENDED_L4));
        break;
    };

    this->height = newHeight;
}

void StateManager::SetArmToDeliver(Queue* queue, MotorController* arm){
    queue->AddTask(new MotorPositionTask(arm, CORAL_ARM_EXTENDED));
}

TargetStateTask::TargetStateTask(RobotState state, StateManager* manager)
{
    this->newState = state;
    this->manager = manager;
}

void TargetStateTask::Start()
{
    manager->targetState = this->newState;
    printf("SettingTargetState: %i, newState: %i (these should be the same)\n",manager->targetState, this->newState);
}

bool TargetStateTask::Loop()
{
    return true;
}

CurrentStateTask::CurrentStateTask(RobotState state, StateManager* manager)
{
    this->newState = state;
    this->manager = manager;
}

void CurrentStateTask::Start()
{
    manager->currentState = this->newState;
}

bool CurrentStateTask::Loop()
{
    return true;
}
