#pragma once
#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::controls;


class MotorController
{
public:
    TalonFX motor;

    PositionVoltage positionRequest  {0_tr};
    MotionMagicVoltage motionMagicRequest  {0_tr};
    VoltageOut voltageRequest{0_V};

    bool useMotionMagic = false;

    double targetPosition = 0.0;
    double targetVoltage = 0.0;

    MotorController(int id, TalonFXConfiguration config, double initPosition = 0.0, std::string canbus = "");
    void Enable();
    void Disable();
    bool GetEnabled();
    void SetTargetPosition(double position);
    void SetVoltage(double voltage);

    void Update();

    bool enabled = true;
};

class MotorFollower
{
public:
    TalonFX motor;

    Follower follower;

    MotorFollower(int id, int idToFollow, bool opposeDirection, std::string canbus = "");

    void Update();
};
