#include "subsystems/RobotState.h"

#include <frc/smartdashboard/SmartDashboard.h>

RobotState::RobotState()
{

}

void RobotState::Periodic()
{
    frc::SmartDashboard::PutNumber("Robot State - Current", this->currentState);
    frc::SmartDashboard::PutNumber("Robot State - Target", this->targetState);
}

frc2::CommandPtr RobotState::SetCurrentState(State state)
{
    return this->RunOnce([this, state] {
        this->currentState = state;
        printf("Setting current to %d\n", state);
    }).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}

frc2::CommandPtr RobotState::SetTargetState(State state)
{
    return this->RunOnce([this, state] {
        this->targetState = state;
        printf("Setting target to %d\n", state);
    }).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}
