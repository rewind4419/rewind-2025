#include "Hardware.h"

#include "Config.h"

#include "ctre/phoenix6/configs/Configs.hpp"

using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::signals;

Hardware::Hardware()
{
    TalonFXConfiguration elevatorConfig;

    elevatorConfig.Slot0.kP = 2.0;
    elevatorConfig.Slot0.kG = 0.5;

    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 40_A;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 40_A;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = 0_A;

    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;

    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 80_tps;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 160_tr_per_s_sq;
    elevatorConfig.MotionMagic.MotionMagicJerk = 1600_tr_per_s_cu;

    elevatorConfig.Feedback.SensorToMechanismRatio = 1;

    elevator = new MotorController(ELEVATOR_MOTOR_1_ID, elevatorConfig, 0.0);
    elevator->useMotionMagic = true;
    elevator2 = new MotorFollower(ELEVATOR_MOTOR_2_ID, ELEVATOR_MOTOR_1_ID, true);
}

void Hardware::Update()
{
    elevator->Update();
    //elevator2->Update();
}

Hardware::~Hardware()
{
    delete elevator;
    delete elevator2;
}
