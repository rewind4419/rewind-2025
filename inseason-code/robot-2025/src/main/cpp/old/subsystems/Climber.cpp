/*
#include "subsystems/Climber.h"

#include <stdio.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/PS4Controller.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandPS4Controller.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "math.h"

configs::TalonFXConfiguration talonFXConfigs{};

using namespace ctre::phoenix6;

Climber::Climber() {
    printf("Initialized Climber\n");

    this->climberMotor.SetPosition(0_tr);

   // this->climberMotor.GetConfigurator().Apply(talonFXConfigs);
}

void Climber::Periodic() {
    
}

frc2::CommandPtr Climber::ClimberResetPosition()
{
    return this->RunOnce([this] {
        this->climberMotor.SetPosition(0.0_tr);
    });
}
*/