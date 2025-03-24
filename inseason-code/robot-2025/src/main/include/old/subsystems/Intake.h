/*
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <rev/SparkMax.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Config.h"

class Intake : public frc2::SubsystemBase {
public:
    Intake();
    void Periodic() override;  
    frc2::CommandPtr IntakeIncreaseSpeed(double speed);
   
private:
 ctre::phoenix6::hardware::TalonFX motor1 {};    
}
*/