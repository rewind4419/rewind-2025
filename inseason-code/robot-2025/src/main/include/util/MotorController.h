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


        MotorController(int id, TalonFXConfiguration config, std::string canbus = "");
        void enable();
        void disable();
        bool getEnabled();
        void setTargetPosition(double position);
        void setVoltage(double voltage);

        void update();

    private:
        bool enabled = false;
};