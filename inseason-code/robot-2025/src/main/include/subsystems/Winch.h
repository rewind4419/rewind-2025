// #pragma once

// #include <frc2/command/SubsystemBase.h>
// #include <frc2/command/CommandPtr.h>

// #include <units/length.h>

// #include <ctre/phoenix6/TalonFX.hpp>

// #include "Config.h"

// class Winch : public frc2::SubsystemBase {
// public:
//     Winch();

//     void Periodic() override;

//     frc2::CommandPtr DrivePower(float power);

// private:
//     // rev::spark::SparkMax coralIntakeMotorL {CORAL_INTAKE_MOTOR_1_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};
//     // rev::spark::SparkMax coralIntakeMotorR {CORAL_INTAKE_MOTOR_2_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};

//     ctre::phoenix6::hardware::TalonFX winchMotor {CORAL_ARM_MOTOR_ID, "rio"};

//     //ctre::phoenix6::controls::MotionMagicVoltage coralArmRequest {0_tr};
// };