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

enum DeliverHeight
{
    DELIVER_ZERO,
    DELIVER_LOW,
    DELIVER_MID,
    DELIVER_HIGH
};

class RobotState : public frc2::SubsystemBase
{
public:
    RobotState();

    void Periodic() override;

    State currentState = STATE_NEUTRAL;
    // State targetState = STATE_NEUTRAL;

    DeliverHeight deliverHeight = DELIVER_ZERO;
    
    frc2::CommandPtr SetCurrentState(State state);
    
    frc2::CommandPtr SetDeliverHeight(DeliverHeight height);
    frc2::CommandPtr IncrementDeliverHeight();
    frc2::CommandPtr DecrementDeliverHeight();

    units::angle::turn_t GetDeliverHeight();
    // frc2::CommandPtr SetTargetState(State state);
private:
};
