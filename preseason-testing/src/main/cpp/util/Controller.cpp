#include "util/Controller.h"

bool Controller::GetDPadUp() {int p = this->GetPOV(); if (p == 0 || p == 45 || p == 315) {return true;} else {return false;}};
bool Controller::GetDPadDown() {};
bool Controller::GetDPadLeft() {};
bool Controller::GetDPadRight() {};
