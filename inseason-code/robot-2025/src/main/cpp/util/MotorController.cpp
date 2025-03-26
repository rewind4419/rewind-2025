#include "util/MotorController.h"
#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::controls;



MotorController::MotorController(int id, TalonFXConfiguration config, double initPosition, std::string canbus) : motor(id,canbus)
{
    motor.GetConfigurator().Apply(config);
    motor.SetPosition(initPosition * 1_tr);
    this->targetPosition = initPosition;
}

void MotorController::Enable()
{
    enabled = true;
}
void MotorController::Disable()
{
    targetVoltage = 0.0;
    enabled = false;
}
bool MotorController::GetEnabled()
{
    return enabled;
}
void MotorController::SetTargetPosition(double position)
{
    targetPosition = position;
}
void MotorController::SetVoltage(double voltage)
{
    targetVoltage = voltage;
}

void MotorController::Update()
{
    if(enabled)
    {
        if(!useMotionMagic)
        {
            motor.SetControl(positionRequest.WithPosition(units::angle::turn_t(targetPosition)));
            printf("%f gerber\n", targetPosition);
        }
        else
        {
            motor.SetControl(motionMagicRequest.WithPosition(units::angle::turn_t(targetPosition)));
        }
    }
    else
    {
        motor.SetControl(voltageRequest.WithOutput(units::voltage::volt_t(targetVoltage)));
    }
}

MotorFollower::MotorFollower(int id, int idToFollow, bool opposeDirection, std::string canbus) : motor(id, canbus), follower(idToFollow, opposeDirection)
{

}

void MotorFollower::Update()
{
    motor.SetControl(follower);
}
