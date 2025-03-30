#include "Hardware.h"

#include "Config.h"

#include "ctre/phoenix6/configs/Configs.hpp"

#include <units/angle.h>

using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::signals;

Hardware::Hardware()
{
    TalonFXConfiguration elevatorConfig;

    elevatorConfig.Slot0.kP = 2.0;
    elevatorConfig.Slot0.kG = 0.5;

    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 40_A;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40_A;

    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;

    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 80_tps;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 160_tr_per_s_sq;
    elevatorConfig.MotionMagic.MotionMagicJerk = 1600_tr_per_s_cu;

    elevatorConfig.Feedback.SensorToMechanismRatio = 1;

    elevator = new MotorController(ELEVATOR_MOTOR_1_ID, elevatorConfig, ELEVATOR_MIN, CTRL_PID_POSITION_MOTION_MAGIC);
    elevator2 = new MotorFollower(ELEVATOR_MOTOR_2_ID, ELEVATOR_MOTOR_1_ID, true);

    TalonFXConfiguration armConfig;

    armConfig.Slot0.kP = 40;
    armConfig.Slot0.kG = 0.7;
    armConfig.Slot0.GravityType = GravityTypeValue::Arm_Cosine;

    armConfig.MotionMagic.MotionMagicCruiseVelocity = 100_tps;
    armConfig.MotionMagic.MotionMagicAcceleration = 1600_tr_per_s_sq;
    armConfig.MotionMagic.MotionMagicJerk = 6400_tr_per_s_cu;

    armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    armConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.CurrentLimits.SupplyCurrentLimit = 40_A;
    armConfig.CurrentLimits.SupplyCurrentLowerLimit = 40_A;

    armConfig.Feedback.SensorToMechanismRatio = 33.333333333;

    armConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;

    arm = new MotorController(CORAL_ARM_MOTOR_ID, armConfig, CORAL_ARM_MIN, CTRL_PID_POSITION_MOTION_MAGIC);

    TalonFXConfiguration wristConfig;

    wristConfig.Slot0.kP = 16;

    wristConfig.MotionMagic.MotionMagicCruiseVelocity = 100_tps; //Changed motor polarity to accomadate new wrist gearbox 3-29-2025
    wristConfig.MotionMagic.MotionMagicAcceleration = 1600_tr_per_s_sq;
    wristConfig.MotionMagic.MotionMagicJerk = 6400_tr_per_s_cu;

    wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.CurrentLimits.SupplyCurrentLimit = 40_A;
    wristConfig.CurrentLimits.SupplyCurrentLowerLimit = 40_A;

    wristConfig.Feedback.SensorToMechanismRatio = 4.0;

    wristConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;

    wrist = new MotorController(CORAL_WRIST_MOTOR_ID, wristConfig, CORAL_WRIST_MIN, CTRL_PID_POSITION_MOTION_MAGIC);
    wrist->SetTargetPosition(CORAL_WRIST_FUNNEL);

    TalonFXConfiguration winchConfig;

    winchConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    winchConfig.CurrentLimits.StatorCurrentLimit = 120_A;
    winchConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    winchConfig.CurrentLimits.SupplyCurrentLimit = 40_A;
    winchConfig.CurrentLimits.SupplyCurrentLowerLimit = 40_A;

    winchConfig.Slot0.kP = 1.0;

    winchConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;
    winch = new MotorController(WINCH_MOTOR_ID, winchConfig, 0.0);

    TalonFXConfiguration intakeConfig;

    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 80_A;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 20_A;
    intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 20_A;

    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;
    intakeConfig.Slot0.kP = 0.3;

    intake = new MotorController(CORAL_INTAKE_MOTOR_ID, intakeConfig, 0.0, CTRL_PID_VELOCITY);

    TalonFXConfiguration flipperConfig;

    flipperConfig.Slot0.kP = 3.5;

    flipperConfig.Feedback.SensorToMechanismRatio = 5.0;

    flipperConfig.MotorOutput.Inverted = InvertedValue::Clockwise_Positive;
    flipperConfig.MotorOutput.NeutralMode = NeutralModeValue::Brake;

    flipper = new MotorController(FUNNEL_FLIPPER_ID, flipperConfig, 0.0, CTRL_PID_POSITION);
}

void Hardware::Update()
{
    elevator->Update();
    elevator2->Update();

    arm->Update();

    wrist->Update();

    winch->Update();

    intake->Update();

    flipper->Update();
}

Hardware::~Hardware()
{
    delete elevator;
    delete elevator2;

    delete arm;

    delete wrist;

    delete winch;

    delete intake;

    delete flipper;
}
