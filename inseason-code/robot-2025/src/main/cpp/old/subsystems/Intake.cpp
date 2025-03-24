/*
#include "subsystems/Intake.h"

#include <stdio.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/PS4Controller.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandPS4Controller.h>

Intake::Intake(){
    printf("Initialize Intake Handler\n");

    this->intakeMotor.SetPosition(0_tr);    
}  

frc2::CommandPtr Intake::IntakeIncreaseSpeed(double speed)
{
    return this->StartEnd(
    [this, speed] {
        // On task start
        printf("Start \n");
        intakeMotor.Set(speed);        
    }, 
    [this] {
        printf("End \n");
        intakeMotor.Set(0.0);        
    });
}
*/