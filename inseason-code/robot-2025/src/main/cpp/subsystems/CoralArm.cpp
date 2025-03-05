#include "subsystems/CoralArm.h"

#include <stdio.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/PS4Controller.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandPS4Controller.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "math.h"

using namespace ctre::phoenix6;

configs::TalonFXConfiguration abctalonFXConfigs{};

CoralArm::CoralArm() {
    printf("Initialized coral arm\n");

    this->coralArmMotor.SetPosition(-0.25_tr);

    // in init function
    
    // set slot 0 gains
    auto& slot0Configs = abctalonFXConfigs.Slot0;
    // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    // slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 100.0; // A position error of 2.5 rotations results in 12 V output
    // slot0Configs.kI = 0; // no output for integrated error
    // slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    //slot0Configs.kG = 3.0;

    // set Motion Magic settings
    auto& motionMagicConfigs = abctalonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100_tps; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 1600_tr_per_s_sq; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 6400_tr_per_s_cu; // Target jerk of 1600 rps/s/s (0.1 seconds)

    //this->coralArmMotor.GetConfigurator().Apply(talonFXConfigs);

    configs::FeedbackConfigs feedback;

    feedback.SensorToMechanismRatio = 33.333333333;

    this->coralArmMotor.GetConfigurator().Apply(abctalonFXConfigs);
    this->coralArmMotor.GetConfigurator().Apply(feedback);
}

void CoralArm::Periodic() {
    frc::SmartDashboard::PutNumber("Arm Position", this->coralArmMotor.GetPosition().GetValueAsDouble());

    // auto& slot0Configs = abctalonFXConfigs.Slot0;
    // // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    // // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    // // slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    // slot0Configs.kP = frc::SmartDashboard::GetNumber("ArmKP", 0.0); // A position error of 2.5 rotations results in 12 V output
    // // slot0Configs.kI = 0; // no output for integrated error
    // // slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    // slot0Configs.kG = frc::SmartDashboard::GetNumber("ArmKG", 0.0);

    // this->coralArmMotor.GetConfigurator().Apply(abctalonFXConfigs);
}


frc2::CommandPtr CoralArm::CoralArmResetPosition()
{
    return this->RunOnce([this] {
        this->coralArmMotor.SetPosition(-0.25_tr);
    });
}

// angle is in radians, where 0 straight up
frc2::CommandPtr CoralArm::CoralArmTo(units::angle::turn_t target)
{
    return this->Run([this, target] {
        printf("Driving to %f\n", target.value());
        this->coralArmMotor.SetControl(
            this->coralArmRequest.WithPosition(
                target
            )
        );
    });
}

frc2::CommandPtr CoralArm::CoralArmRun(double speed) {

    return this->StartEnd(
    [this, speed] {
        // On task start
        printf("Start \n");
        coralIntakeMotorL.Set(speed);
        coralIntakeMotorR.Set(-speed);
    }, 
    [this] {
        printf("End \n");
        // On task end/cancel
        coralIntakeMotorL.Set(0.0);
        coralIntakeMotorR.Set(0.0);
    });
}
