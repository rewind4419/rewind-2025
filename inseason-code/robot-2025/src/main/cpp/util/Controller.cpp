#include "util/Controller.h"

bool Controller::GetDPadUp() {int p = this->GetPOV(); if (p == 0 || p == 45 || p == 315) {return true;} else {return false;}};
bool Controller::GetDPadDown() {int p = this->GetPOV(); if (p == 135 || p == 180 || p == 225) {return true;} else {return false;}};
bool Controller::GetDPadLeft() {int p = this->GetPOV(); if (p == 225 || p == 270 || p == 315) {return true;} else {return false;}};
bool Controller::GetDPadRight() {int p = this->GetPOV(); if (p == 45 || p == 90 || p == 135) {return true;} else {return false;}};

bool Controller::GetDPadUpPressed()
{
    if (GetDPadUp() == true && DPadUpLast == false) {return true;} else {return false;}
}

bool Controller::GetDPadDownPressed()
{
    if (GetDPadDown() == true && DPadDownLast == false) {return true;} else {return false;}
}

bool Controller::GetDPadLeftPressed()
{
    if (GetDPadLeft() == true && DPadLeftLast == false) {return true;} else {return false;}
}

bool Controller::GetDPadRightPressed()
{
    if (GetDPadRight() == true && DPadRightLast == false) {return true;} else {return false;}
}

void Controller::Update()
{
    DPadUpLast = GetDPadUp();
    DPadDownLast = GetDPadDown();
    DPadLeftLast = GetDPadLeft();
    DPadRightLast = GetDPadRight();
}
