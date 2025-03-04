#include "subsystems/Winch.h"
#include <frc2/command/FunctionalCommand.h>

Winch::Winch()
{
    
}

void Winch::Periodic()
{

}

frc2::CommandPtr Winch::DrivePower(std::function<float()> powerProvider){
    return this->Run([this, powerProvider]{
        float power = powerProvider();
        printf("Setting to de %f\n", power);
        this->winchMotor.SetControl(winchMotorRequest.WithOutput(power));
    });
}

// frc2::CommandPtr Winch::HoldPos(units::angle::turn_t position){
//     return this->Run([this, position]{
//         this->winchMotor.SetControl(winchPosition.WithPosition(position));
//     });
// }

frc2::CommandPtr Winch::GotoPosition(units::angle::turn_t position){
    return frc2::FunctionalCommand(
            [this, position] () {this->target = position;},
            [this, position] () {this->winchMotor.SetControl(winchPosition.WithPosition(position));},
            [] (bool interrupted) {},
            [this, position] () -> bool {return fabsf(this->winchMotor.GetPosition().GetValueAsDouble() - position.value()) < 1.0;},
            {this}
    ).ToPtr();
}