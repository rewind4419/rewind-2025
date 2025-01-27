#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/length.h>

class Elevator : public frc2::SubsystemBase
{
public:
    Elevator();

    // Height is in meters
    frc2::CommandPtr SetHeight(float height);
private:

};
