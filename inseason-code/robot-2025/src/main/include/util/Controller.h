#pragma once

#include <frc/PS4Controller.h>


class RewindController : public frc::PS4Controller
{
public:
    // adds the constructor from PS4Controller to this one
    using frc::PS4Controller::PS4Controller;

    // Any custom game controller functions below
    bool GetDPadUp();
    bool GetDPadDown();
    bool GetDPadLeft();
    bool GetDPadRight();
};
