#pragma once

#include "queue/Queue.h"

enum RobotState
{
    STATE_NEUTRAL,
    STATE_DELIVER,
    STATE_FUNNEL,
    STATE_CLIMB
};

class StateManager
{
public:
    RobotState currentState = STATE_NEUTRAL;
    RobotState targetState = STATE_NEUTRAL;
};

class TargetStateTask : public Task
{
public:
    StateManager* manager;
    RobotState newState;
    TargetStateTask(RobotState state, StateManager* manager);

    void Start() override;
    bool Loop() override;
};

class CurrentStateTask : public Task
{
public:
    StateManager* manager;
    RobotState newState;
    CurrentStateTask(RobotState state, StateManager* manager);

    void Start() override;
    bool Loop() override;
};
