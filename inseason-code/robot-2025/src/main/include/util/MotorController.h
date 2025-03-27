#pragma once
#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::controls;

enum MotorControllerMode
{
    CTRL_PID_POSITION,
    CTRL_PID_POSITION_MOTION_MAGIC,
    CTRL_PID_VELOCITY,
    CTRL_VOLTAGE
};

class MotorController
{
public:
    TalonFX motor;

    PositionVoltage positionRequest  {0_tr};
    MotionMagicVoltage motionMagicRequest  {0_tr};
    VelocityVoltage velocityRequest {0_tps};
    VoltageOut voltageRequest{0_V};

    MotorControllerMode mode;

    double targetPosition = 0.0;
    double targetVelocity = 0.0;
    double targetVoltage = 0.0;

    MotorController(int id, TalonFXConfiguration config, double initPosition = 0.0, MotorControllerMode mode = CTRL_PID_POSITION, std::string canbus = "");

    void SetTargetPosition(double position);
    void SetTargetVelocity(double velocity);
    void SetVoltage(double voltage);

    void SetMode(MotorControllerMode mode);

    void Update();
};

class MotorFollower
{
public:
    TalonFX motor;

    Follower follower;

    MotorFollower(int id, int idToFollow, bool opposeDirection, std::string canbus = "");

    void Update();
};
