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
    });
}

frc2::CommandPtr RobotState::SetTargetState(State state)
{
    return this->RunOnce([this, state] {
        this->targetState = state;
    });
}
