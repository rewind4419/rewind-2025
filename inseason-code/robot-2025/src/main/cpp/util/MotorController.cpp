#include "util/MotorController.h"
#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::controls;



MotorController::MotorController(int id, TalonFXConfiguration config, std::string canbus = "") : motor(id,canbus)
{
    motor.GetConfigurator().Apply(config);
}

void MotorController::enable()
{
    enabled = true;
}
void MotorController::disable()
{
    targetVoltage = 0.0;
    enabled = false;
}
bool MotorController::getEnabled()
{
    return enabled;
}
void MotorController::setTargetPosition(double position)
{
    targetPosition = position;
}
void MotorController::setVoltage(double voltage)
{
    targetVoltage = voltage;
}

void MotorController::update()
{
    if(enabled)
    {
        if(!useMotionMagic)
        {
            motor.SetControl(positionRequest.WithPosition(units::angle::turn_t(targetPosition)));
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