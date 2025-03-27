#include "State.h"

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
