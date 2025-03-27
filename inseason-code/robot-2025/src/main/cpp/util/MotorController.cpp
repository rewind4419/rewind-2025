#include "util/MotorController.h"
#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::controls;

MotorController::MotorController(int id, TalonFXConfiguration config, double initPosition, MotorControllerMode mode, std::string canbus) : motor(id,canbus)
{
    motor.GetConfigurator().Apply(config);
    motor.SetPosition(initPosition * 1_tr);
    this->targetPosition = initPosition;
    this->mode = mode;
}

void MotorController::SetTargetPosition(double position)
{
    targetPosition = position;
}
void MotorController::SetTargetVelocity(double velocity)
{
    targetVelocity = velocity;
}
void MotorController::SetVoltage(double voltage)
{
    targetVoltage = voltage;
}

void MotorController::SetMode(MotorControllerMode mode)
{
    this->mode = mode;
}

void MotorController::Update()
{
    switch (mode)
    {
    case CTRL_PID_POSITION:
        motor.SetControl(positionRequest.WithPosition(units::angle::turn_t(targetPosition)));
        break;
    case CTRL_PID_POSITION_MOTION_MAGIC:
        motor.SetControl(motionMagicRequest.WithPosition(units::angle::turn_t(targetPosition)));
        break;
    case CTRL_PID_VELOCITY:
        motor.SetControl(velocityRequest.WithVelocity(units::angular_velocity::turns_per_second_t(targetVelocity)));
        break;
    case CTRL_VOLTAGE:
        motor.SetControl(voltageRequest.WithOutput(units::voltage::volt_t(targetVoltage)));
        break;
    default:
        printf("Warning, unimplemented PID mode! Defaulting to CTRL_PID_POSITION\n");
        motor.SetControl(positionRequest.WithPosition(units::angle::turn_t(targetPosition)));
        break;
    }
}

MotorFollower::MotorFollower(int id, int idToFollow, bool opposeDirection, std::string canbus) : motor(id, canbus), follower(idToFollow, opposeDirection)
{

}

void MotorFollower::Update()
{
    motor.SetControl(follower);
}
