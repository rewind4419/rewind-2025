#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <units/length.h>

#include <rev/SparkMax.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Config.h"

class CoralArm : public frc2::SubsystemBase {
public:
    CoralArm();

    void Periodic() override;

    frc2::CommandPtr CoralArmTo(float angle);
    frc2::CommandPtr CoralArmResetPosition();

    // Runs the Coral Intake motors at the specified POWER until the task is canceled
    frc2::CommandPtr CoralArmRun(double speed);
private:
    // rev::spark::SparkMax coralIntakeMotorL {CORAL_INTAKE_MOTOR_1_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    // rev::spark::SparkMax coralIntakeMotorR {CORAL_INTAKE_MOTOR_2_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};

    ctre::phoenix6::hardware::TalonFX coralArmMotor {CORAL_ARM_MOTOR_ID, "rio"};

    //ctre::phoenix6::controls::MotionMagicVoltage coralArmRequest {0_tr};
};