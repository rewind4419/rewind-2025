/*
#include "subsystems/AlgaeHandler.h"

#include <stdio.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/PS4Controller.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandPS4Controller.h>

AlgaeHandler::AlgaeHandler(){
    printf("Initialize Algae Handler\n");

    this->algaeHandlerMotor1.SetPosition(0_tr);
    this->algaeHandlerMotor2.SetPosition(0_tr);
}  

frc2::CommandPtr AlgaeHandler::AlgaeHandlerResetPosition()
{
    return this->RunOnce([this] {
        this->algaeHandlerMotor1.SetPosition(0.0_tr);
        this->algaeHandlerMotor2.SetPosition(0.0_tr);
    });
}

// angle is in radians, where 0 straight up
frc2::CommandPtr AlgaeHandler::AlgaeHandlerTo(float angle)
{
    this->RunOnce([this, angle] {
        // this->algaeHandlerMotor.SetControl(
        //     this->algaeHandlerRequest.WithPosition(
        //         units::angle::turn_t{
        //             angle / (2.0f * M_PI)
        //         }
        //     )
        // );
    });
}
*/