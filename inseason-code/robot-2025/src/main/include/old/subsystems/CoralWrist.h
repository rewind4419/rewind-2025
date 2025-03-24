#pragma once


#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/FunctionalCommand.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <units/length.h>
#include "Config.h"



class CoralWrist : public frc2::SubsystemBase {
public:
    CoralWrist();

    void Periodic() override;

    frc2::CommandPtr SetWristVelocity(units::angular_velocity::turns_per_second_t speed);

    frc2::CommandPtr ResetPosition();
    frc2::CommandPtr HoldPos(std::function<units::angle::turn_t()> offset);

    // If wait is true, the task doesn't finish until the arm reaches its target.
    // If wait is false, the task tells the arm to start moving and then finishes immediately.
    // Wai\t is true by default, so if you don't specify, it will wait.
    frc2::CommandPtr SetPosition(units::angle::turn_t pos, bool wait = true);
    frc2::CommandPtr SetPositionProvider(std::function<units::angle::turn_t()> pos, bool wait = true);

    units::angle::turn_t target;

    const float epsilon = 0.06f; // in turns

public:
    // units::angular_velocity::turns_per_second_t maxspeed {0.2_tps};
    // units::angle::turn_t pos;
    ctre::phoenix6::hardware::TalonFX coralWristMotor {CORAL_WRIST_MOTOR_ID};

    ctre::phoenix6::controls::VelocityVoltage coralWristVelRequest {0_tps};
    ctre::phoenix6::controls::MotionMagicVoltage coralWristPosRequest {0_tr};
};