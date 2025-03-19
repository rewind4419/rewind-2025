#include "subsystems/CoralWrist.h"
#include "util/maths.h"
#include "math.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <units/angle.h>
#include <units/length.h>
#include <units/angular_velocity.h>

// CoralWrist::CoralWrist(){
//     printf("Initialized wrist\n");
//     this->pos = coralWristMotor.GetPosition().GetValue();
// }
// frc2::CommandPtr CoralWrist::SetWristVelocity(units::angular_velocity::turns_per_second_t speed){
//     return this->Run([this, speed] {
//         // // Commented out for now, there is some weird compile error from here involving unit conversions - Sherwin
//         // // I also commented out the CoralWrist from RobotContainer.h and all references to it in RobotContainer.cpp
//         // this->coralWristMotor.SetControl(coralWristVelRequest.WithVelocity(speed * maxspeed));
//         // this->pos = coralWristMotor.GetPosition().GetValue();
//     }).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
// }

using namespace ctre::phoenix6;

configs::TalonFXConfiguration coralWristTalonFXConfigs{}; //Arm Pivot

CoralWrist::CoralWrist() {
    printf("Initialized coral arm\n");

    this->coralWristMotor.SetPosition(CORAL_WRIST_MIN);

    this->target = CORAL_WRIST_FUNNEL;

    // in init function
    
    // set slot 0 gains
    auto& slot0Configs = coralWristTalonFXConfigs.Slot0;
    // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    // slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 16; // A position error of 2.5 rotations results in 12 V output
    // slot0Configs.kI = 0; // no output for integrated error
    // slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    //slot0Configs.kG = 0.7;
    //slot0Configs.GravityType = signals::GravityTypeValue::Arm_Cosine;

    // set Motion Magic settings
    auto& motionMagicConfigs = coralWristTalonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100_tps; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 1600_tr_per_s_sq; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 6400_tr_per_s_cu; // Target jerk of 1600 rps/s/s (0.1 seconds)

    //this->coralArmMotor.GetConfigurator().Apply(talonFXConfigs);

    configs::FeedbackConfigs feedback;

    feedback.SensorToMechanismRatio = 4.0;


    this->coralWristMotor.GetConfigurator().Apply(coralWristTalonFXConfigs);
    this->coralWristMotor.GetConfigurator().Apply(feedback);
}

void CoralWrist::Periodic() {
    frc::SmartDashboard::PutNumber("Wrist Position", this->coralWristMotor.GetPosition().GetValueAsDouble());

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


frc2::CommandPtr CoralWrist::ResetPosition()
{
    return this->RunOnce([this] {
        printf("Setting pos\n");
        this->coralWristMotor.SetPosition(CORAL_WRIST_MIN);
    }).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}

frc2::CommandPtr CoralWrist::HoldPos(std::function<units::angle::turn_t()> offset)
{
    return this->Run([this, offset]{
        //printf("Holding\n");
        this->coralWristMotor.SetControl(coralWristPosRequest.WithPosition(clamp(target + offset(), CORAL_WRIST_MIN, CORAL_WRIST_MAX)));
    });
}

frc2::CommandPtr CoralWrist::SetPosition(units::angle::turn_t pos, bool wait)
{
    return frc2::FunctionalCommand(
        [this, pos] () {
            printf("Started coral pos\n");
            this->target = pos;
        },
        [this, pos] () {
            this->coralWristMotor.SetControl(coralWristPosRequest.WithPosition(clamp(pos, CORAL_ARM_MIN, CORAL_ARM_MAX)));
        },
        [] (bool interrupted) {
            printf("Finished coral pos\n");
        },
        [this, pos, wait] () -> bool {
            if (wait == false) {return true;}
            //printf("Arm distance %f\n", this->motor1.GetPosition().GetValueAsDouble() - pos.value());
            bool done = fabsf(this->coralWristMotor.GetPosition().GetValueAsDouble() - pos.value()) < this->epsilon;
            return done;
        },
        {this}
    ).ToPtr().WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}

frc2::CommandPtr CoralWrist::SetPositionProvider(std::function<units::angle::turn_t()> pos, bool wait)
{
    return frc2::FunctionalCommand(
        [this, pos] () {
            this->target = pos();
        },
        [this, pos] () {
            this->coralWristMotor.SetControl(coralWristPosRequest.WithPosition(clamp(pos(), CORAL_ARM_MIN, CORAL_ARM_MAX)));
        },
        [] (bool interrupted) {},//printf("Finished going!\n");},
        [this, pos, wait] () -> bool {
            if (wait == false) {return true;}
            //printf("Wrist distance %f\n", this->coralWristMotor.GetPosition().GetValueAsDouble() - pos().value());
            bool done = fabsf(this->coralWristMotor.GetPosition().GetValueAsDouble() - pos().value()) < this->epsilon;
            return done;
        },
        {this}
    ).ToPtr();
}


