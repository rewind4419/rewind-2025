#include "subsystems/CoralArm.h"

#include <stdio.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/PS4Controller.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandPS4Controller.h>

frc2::CommandPtr CoralArm::CoralArmRun(double speed){

    this->StartEnd([this, speed] () {
    motorR.Set(speed);
    motorL.Set(speed * -1.0f);
    }, [this] {
        motorL.Set(0.0);
        motorR.Set(0.0);
    });
}

CoralArm::CoralArm(){

}
void CoralArm::Periodic(){
    
}