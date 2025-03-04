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

CoralArm::CoralArm() {
    printf("Initialized coral arm\n");

    this->coralArmMotor.SetPosition(0_tr);

    // // in init function
    // configs::TalonFXConfiguration talonFXConfigs{};

    // // set slot 0 gains
    // auto& slot0Configs = talonFXConfigs.Slot0;
    // // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    // // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    // // slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    // slot0Configs.kP = 0.0; // A position error of 2.5 rotations results in 12 V output
    // // slot0Configs.kI = 0; // no output for integrated error
    // // slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // // set Motion Magic settings
    // auto& motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = 1_tps; // Target cruise velocity of 80 rps
    // motionMagicConfigs.MotionMagicAcceleration = 16_tr_per_s_sq; // Target acceleration of 160 rps/s (0.5 seconds)
    // motionMagicConfigs.MotionMagicJerk = 160_tr_per_s_cu; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // //this->coralArmMotor.GetConfigurator().Apply(talonFXConfigs);
}

void CoralArm::Periodic() {
    
}

frc2::CommandPtr CoralArm::CoralArmResetPosition()
{
    return this->RunOnce([this] {
        this->coralArmMotor.SetPosition(0.0_tr);
    });
}

// angle is in radians, where 0 straight up
frc2::CommandPtr CoralArm::CoralArmTo(float angle)
{
    printf("Yeeting\n");
    this->RunOnce([this, angle] {
        // this->coralArmMotor.SetControl(
        //     this->coralArmRequest.WithPosition(
        //         units::angle::turn_t{
        //             angle / (2.0f * M_PI)
        //         }
        //     )
        // );
    });
}

// frc2::CommandPtr CoralArm::CoralArmRun(double speed) {

//     this->StartEnd(
//     [this, speed] {
//         // On task start
//         coralIntakeMotorL.Set(speed);
//         coralIntakeMotorR.Set(-speed);
//     }, 
//     [this] {
//         // On task end/cancel
//         coralIntakeMotorL.Set(0.0);
//         coralIntakeMotorR.Set(0.0);
//     });
// }
