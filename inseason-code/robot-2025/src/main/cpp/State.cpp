#include "State.h"

#include "util/maths.h"
#include "queue/FrcTasks.h"
#include "Config.h"

void StateManager::IncrementDeliverHeight()
{
    this->height = (DeliverHeight)clamp(this->height + 1, HEIGHT_ZERO, HEIGHT_L4);
}

void StateManager::DecrementDeliverHeight()
{
    this->height = (DeliverHeight)clamp(this->height - 1, HEIGHT_ZERO, HEIGHT_L4);
}

void StateManager::GoToDeliverHeight(Queue* queue, MotorController* elevator, MotorController* wrist)
{
    switch (this->height)
    {
    case HEIGHT_ZERO:
        queue->AddTask(new MotorPositionTask(elevator, ELEVATOR_MIN));
        queue->AddTask(new MotorPositionTask(wrist, CORAL_WRIST_EXTENDED));
        break;
    case HEIGHT_L2:
        queue->AddTask(new MotorPositionTask(elevator, ELEVATOR_DELIVER_LOW));
        queue->AddTask(new MotorPositionTask(wrist, CORAL_WRIST_EXTENDED));
        break;
    case HEIGHT_L3:
        queue->AddTask(new MotorPositionTask(elevator, ELEVATOR_DELIVER_MID));
        queue->AddTask(new MotorPositionTask(wrist, CORAL_WRIST_EXTENDED));
        break;
    case HEIGHT_L4:
        queue->AddTask(new MotorPositionTask(elevator, ELEVATOR_DELIVER_HIGH));
        queue->AddTask(new MotorPositionTask(wrist, CORAL_WRIST_EXTENDED));
        break;
    };
}

TargetStateTask::TargetStateTask(RobotState state, StateManager* manager)
{
    this->newState = state;
    this->manager = manager;
}

void TargetStateTask::Start()
{
    manager->targetState = this->newState;
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
