#include "subsystems/CoralArm.h"

#include <stdio.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/PS4Controller.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandPS4Controller.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "util/maths.h"

#include "math.h"

using namespace ctre::phoenix6;

configs::TalonFXConfiguration abctalonFXConfigs{}; //Arm Pivot
configs::TalonFXConfiguration abcdetalonFXConfigs{}; //Wheels

CoralArm::CoralArm() {
    printf("Initialized coral arm\n");

    this->motor1.SetPosition(0_tr);

    // in init function
    
    // set slot 0 gains
    auto& slot0Configs = abctalonFXConfigs.Slot0;
    // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    // slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 40; // A position error of 2.5 rotations results in 12 V output
    // slot0Configs.kI = 0; // no output for integrated error
    // slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.kG = 0.7;
    slot0Configs.GravityType = signals::GravityTypeValue::Arm_Cosine;

    auto& slot1Configs = abcdetalonFXConfigs.Slot0;
    slot1Configs.kP = 0.3;

    // set Motion Magic settings
    auto& motionMagicConfigs = abctalonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100_tps; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 1600_tr_per_s_sq; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 6400_tr_per_s_cu; // Target jerk of 1600 rps/s/s (0.1 seconds)

    //this->coralArmMotor.GetConfigurator().Apply(talonFXConfigs);

    configs::FeedbackConfigs feedback;

    feedback.SensorToMechanismRatio = 33.333333333;

    this->motor1.GetConfigurator().Apply(abctalonFXConfigs);
    this->motor1.GetConfigurator().Apply(feedback);

    this->coralIntakeMotor.GetConfigurator().Apply(abcdetalonFXConfigs);

    this->target = 0.0_tr;
}

void CoralArm::Periodic() {
    frc::SmartDashboard::PutNumber("Arm Position", this->motor1.GetPosition().GetValueAsDouble());

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


frc2::CommandPtr CoralArm::ResetPosition()
{
    return this->RunOnce([this] {
        printf("Setting pos\n");
        this->motor1.SetPosition(0_tr);
    }).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}

frc2::CommandPtr CoralArm::HoldPos()
{
    return this->Run([this]{
        //printf("Holding\n");
        this->motor1.SetControl(coralArmRequest.WithPosition(clamp(target, CORAL_ARM_MIN, CORAL_ARM_MAX)));
    });
}

frc2::CommandPtr CoralArm::SetPosition(units::angle::turn_t pos, bool wait)
{
    return frc2::FunctionalCommand(
        [this, pos] () {
            printf("Started coral pos\n");
            this->target = pos;
        },
        [this, pos] () {
            this->motor1.SetControl(coralArmRequest.WithPosition(clamp(pos, CORAL_ARM_MIN, CORAL_ARM_MAX)));
        },
        [] (bool interrupted) {
            printf("Finished coral pos\n");
        },
        [this, pos, wait] () -> bool {
            if (wait == false) {return true;}
            //printf("Arm distance %f\n", this->motor1.GetPosition().GetValueAsDouble() - pos.value());
            bool done = fabsf(this->motor1.GetPosition().GetValueAsDouble() - pos.value()) < this->epsilon;
            return done;
        },
        {this}
    ).ToPtr().WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}

frc2::CommandPtr CoralArm::SetPositionProvider(std::function<units::angle::turn_t()> pos, bool wait)
{
    return frc2::FunctionalCommand(
        [this, pos] () {
            this->target = pos();
        },
        [this, pos] () {
            this->motor1.SetControl(coralArmRequest.WithPosition(clamp(pos(), CORAL_ARM_MIN, CORAL_ARM_MAX)));
        },
        [] (bool interrupted) {/*printf("Finished going!\n");*/},
        [this, pos, wait] () -> bool {
            if (wait == false) {return true;}
            //printf("Arm distance %f\n", this->motor1.GetPosition().GetValueAsDouble() - pos().value());
            bool done = fabsf(this->motor1.GetPosition().GetValueAsDouble() - pos().value()) < this->epsilon;
            return done;
        },
        {this}
    ).ToPtr();
}

// Run intake, positive pulls in, negative yeets out
frc2::CommandPtr CoralArm::CoralArmRunIntake(units::angular_velocity::turns_per_second_t speed) {

    return this->RunEnd(
    [this, speed] {
        // On task start
        //printf("Start \n");
        //coralIntakeMotorL.Set(speed);
        //coralIntakeMotorR.Set(-speed);
        //coralIntakeMotor.Set(speed);
        this->coralIntakeMotor.SetControl(coralIntakeRequest.WithVelocity(speed));
        //coralIntakeMotor.SetInverted(true); //invert the second set of wheels
    }, 
    [this] {
        //printf("End \n");
        // On task end/cancel
        //coralIntakeMotorL.Set(0.0);
        //coralIntakeMotorR.Set(0.0);
        this->coralIntakeMotor.SetControl(coralIntakeRequest.WithVelocity(0.0_tps));
    });
}

// Base intake speed
frc2::CommandPtr CoralArm::CoralArmBaseIntake(double speed) {
    return this->Run(
    [this, speed] {
        //coralIntakeMotorL.Set(speed);
        //coralIntakeMotorR.Set(-speed);
        //coralIntakeMotor.Set(speed);
        coralIntakeMotor.SetInverted(true); //invert the second set of wheels
    });
}