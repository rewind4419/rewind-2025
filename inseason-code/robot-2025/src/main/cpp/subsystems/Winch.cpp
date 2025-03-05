#include "subsystems/Winch.h"
#include <frc2/command/FunctionalCommand.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/configs/Configs.hpp>

using namespace ctre::phoenix6;

Winch::Winch()
{
    configs::TalonFXConfiguration talonFXConfigs{};

    configs::Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kP = 1.0;

    winchMotor.GetConfigurator().Apply(slot0Configs);

    this->winchMotor.SetPosition(0_tr);
}

void Winch::Periodic()
{
    frc::SmartDashboard::PutNumber("Winch current encoder position", this->winchMotor.GetPosition().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("Winch target", this->target.value());
}

frc2::CommandPtr Winch::DrivePower(std::function<float()> powerProvider){
    return this->Run([this, powerProvider]{
        float power = powerProvider();
        //printf("Setting to de %f\n", power);
        this->winchMotor.SetControl(winchMotorRequest.WithOutput(power));
        this->target = this->winchMotor.GetPosition().GetValue();
    });
}

frc2::CommandPtr Winch::HoldPos(){
    return this->Run([this]{
        //printf("Holding\n");
        this->winchMotor.SetControl(winchPosition.WithPosition(this->target));
    });
}

frc2::CommandPtr Winch::GotoPosition(units::angle::turn_t position){
    return frc2::FunctionalCommand(
            [this, position] () {this->target = position;},
            [this, position] () {this->winchMotor.SetControl(winchPosition.WithPosition(position));},
            [] (bool interrupted) {/*printf("Finished going!\n");*/},
            [this, position] () -> bool {
                printf("Winch distance %f\n", this->winchMotor.GetPosition().GetValueAsDouble() - position.value());
                return fabsf(this->winchMotor.GetPosition().GetValueAsDouble() - position.value()) < 1.0;},
            {this}
    ).ToPtr();
}