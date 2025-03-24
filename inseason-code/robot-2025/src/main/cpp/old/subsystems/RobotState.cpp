#include "subsystems/RobotState.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "util/maths.h"

RobotState::RobotState()
{
    
}

void RobotState::Periodic()
{
    frc::SmartDashboard::PutNumber("Robot State - Current", this->currentState);
    // frc::SmartDashboard::PutNumber("Robot State - Target", this->targetState);
}

frc2::CommandPtr RobotState::SetCurrentState(State state)
{
    return this->RunOnce([this, state] {
        this->currentState = state;
        printf("Setting current to %d\n", state);
    }).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}

frc2::CommandPtr RobotState::SetDeliverHeight(DeliverHeight height)
{
    return this->RunOnce([this, height] {
        this->deliverHeight = height;
        printf("Setting height to %d\n", height);
    }).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}

frc2::CommandPtr RobotState::IncrementDeliverHeight()
{
    return this->RunOnce([this] {
        this->deliverHeight = (DeliverHeight)clamp(this->deliverHeight + 1, DELIVER_ZERO, DELIVER_HIGH);
    }).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}

frc2::CommandPtr RobotState::DecrementDeliverHeight()
{
    return this->RunOnce([this] {
        this->deliverHeight = (DeliverHeight)clamp(this->deliverHeight - 1, DELIVER_ZERO, DELIVER_HIGH);
    }).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}

units::angle::turn_t RobotState::GetDeliverHeight()
{
    switch (deliverHeight)
    {
    case DELIVER_ZERO:
        return ELEVATOR_MIN;
        break;
    case DELIVER_LOW:
        return ELEVATOR_DELIVER_LOW;
        break;
    case DELIVER_MID:
        return ELEVATOR_DELIVER_MID;
        break;
    case DELIVER_HIGH:
        return ELEVATOR_DELIVER_HIGH;
        break;
    default:
        printf("Warning! GetDeliverHeight case not handled, %d\n", deliverHeight);
        return ELEVATOR_MIN;
        break;
    }
}

// frc2::CommandPtr RobotState::SetTargetState(State state)
// {
//     return this->RunOnce([this, state] {
//         this->targetState = state;
//         printf("Setting target to %d\n", state);
//     }).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
// }
