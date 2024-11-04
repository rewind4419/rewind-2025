#pragma once

#include "subsystems/Drivetrain.h"
#include "util/Controller.h"

#include <frc/TimedRobot.h>

class Robot : public frc::TimedRobot {
public:
    Controller driver{0};
    Controller mate{0};

    Drivetrain drivetrain;


    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
};
