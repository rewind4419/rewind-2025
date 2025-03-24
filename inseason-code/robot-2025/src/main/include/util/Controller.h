#include <frc/PS4Controller.h>


class Controller : public frc::PS4Controller
{
public:
    // adds the constructor from PS4Controller to this one
    using frc::PS4Controller::PS4Controller;

    // Any custom game controller functions below
    bool GetDPadUp();
    bool GetDPadDown();
    bool GetDPadLeft();
    bool GetDPadRight();

    bool GetDPadUpPressed();
    bool GetDPadDownPressed();
    bool GetDPadLeftPressed();
    bool GetDPadRightPressed();

    void Update();
private:
    bool DPadUpLast = false;
    bool DPadDownLast = false;
    bool DPadLeftLast = false;
    bool DPadRightLast = false;
};
