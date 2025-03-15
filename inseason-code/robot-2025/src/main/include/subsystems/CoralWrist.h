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

    frc2::CommandPtr SetWristVelocity(units::angular_velocity::turns_per_second_t speed);
    frc2::CommandPtr HoldPos();
    
public:
    units::angular_velocity::turns_per_second_t maxspeed {0.2_tps};
    units::angle::turn_t pos;
    ctre::phoenix6::hardware::TalonFX coralWristMotor {CORAL_WRIST_MOTOR_ID};
    ctre::phoenix6::controls::VelocityVoltage coralWristVelRequest {0_tps};
    ctre::phoenix6::controls::MotionMagicVoltage coralWristPosRequest {0_tr};
};