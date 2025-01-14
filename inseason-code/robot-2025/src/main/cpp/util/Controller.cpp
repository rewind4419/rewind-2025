#include "util/Controller.h"

bool Controller::GetDPadUp() {int p = this->GetPOV(); if (p == 0 || p == 45 || p == 315) {return true;} else {return false;}};
bool Controller::GetDPadDown() {int p = this->GetPOV(); if (p == 135 || p == 180 || p == 225) {return true;} else {return false;}};
bool Controller::GetDPadLeft() {int p = this->GetPOV(); if (p == 225 || p == 270 || p == 315) {return true;} else {return false;}};
bool Controller::GetDPadRight() {int p = this->GetPOV(); if (p == 45 || p == 90 || p == 135) {return true;} else {return false;}};
