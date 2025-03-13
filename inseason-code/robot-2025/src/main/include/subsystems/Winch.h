#pragma once

#include <functional>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <units/length.h>
#include <units/dimensionless.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Config.h"

class Winch : public frc2::SubsystemBase {
public:
    Winch();

    void Periodic() override;

    frc2::CommandPtr GotoPosition(units::angle::turn_t position);
    frc2::CommandPtr HoldPos();
    frc2::CommandPtr DrivePower(std::function<float()> powerProvider);

    frc2::CommandPtr TestCommand();
    frc2::CommandPtr TestCommand2();

    units::angle::turn_t target = 0.0_tr;

    bool iscool = false;
private:
    // rev::spark::SparkMax coralIntakeMotorL {CORAL_INTAKE_MOTOR_1_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    // rev::spark::SparkMax coralIntakeMotorR {CORAL_INTAKE_MOTOR_2_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    ctre::phoenix6::controls::DutyCycleOut winchMotorRequest {0};
    ctre::phoenix6::controls::PositionVoltage winchPosition {0.0_tr};


    ctre::phoenix6::hardware::TalonFX winchMotor {WINCH_MOTOR_ID, "rio"};

    //ctre::phoenix6::controls::MotionMagicVoltage coralArmRequest {0_tr};
};