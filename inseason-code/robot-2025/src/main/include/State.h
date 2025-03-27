#pragma once

#include "queue/Queue.h"

enum RobotState
{
    STATE_NEUTRAL,
    STATE_DELIVER,
    STATE_FUNNEL,
    STATE_CLIMB
};

enum DeliverHeight
{
    HEIGHT_ZERO,
    HEIGHT_L1,
    HEIGHT_L2,
    HEIGHT_L3,
    HEIGHT_L4
};

#define POV_UP 0
#define POV_RIGHT 90
#define POV_DOWN 180
#define POV_LEFT 270

class StateManager
{
public:
    RobotState currentState = STATE_NEUTRAL;
    RobotState targetState = STATE_NEUTRAL;
    DeliverHeight height = HEIGHT_ZERO;
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
