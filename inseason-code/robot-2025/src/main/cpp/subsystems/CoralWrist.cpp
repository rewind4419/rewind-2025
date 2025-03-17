#include "subsystems/CoralWrist.h"
#include "util/maths.h"
#include "math.h"
CoralWrist::CoralWrist(){
    printf("Initialized wrist\n");
    this->pos = coralWristMotor.GetPosition().GetValue();
}
frc2::CommandPtr CoralWrist::SetWristVelocity(units::angular_velocity::turns_per_second_t speed){
    return this->Run([this, speed] {
        // // Commented out for now, there is some weird compile error from here involving unit conversions - Sherwin
        // // I also commented out the CoralWrist from RobotContainer.h and all references to it in RobotContainer.cpp
        // this->coralWristMotor.SetControl(coralWristVelRequest.WithVelocity(speed * maxspeed));
        // this->pos = coralWristMotor.GetPosition().GetValue();
    }).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);
}
frc2::CommandPtr CoralWrist::HoldPos(){
    return this->Run([this]{
        this->coralWristMotor.SetControl(coralWristPosRequest.WithPosition(pos));
    });
}
