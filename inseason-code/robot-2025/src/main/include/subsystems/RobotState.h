#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/length.h>
#include <units/angle.h>

#include "Config.h"

enum State
{
    STATE_NEUTRAL,
    STATE_FUNNEL,
    STATE_DELIVER_LOW
};

class RobotState : public frc2::SubsystemBase
{
public:
    RobotState();

    void Periodic() override;

    State currentState = STATE_NEUTRAL;
    State targetState = STATE_NEUTRAL;
    
    frc2::CommandPtr SetCurrentState(State state);
    frc2::CommandPtr SetTargetState(State state);
private:
};
