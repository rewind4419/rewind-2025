#pragma once

#include "subsystems/Drivetrain.h"
#include "util/Controller.h"
#include <frc/TimedRobot.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc/RobotController.h>

enum AutoType
{
    AUTO_FAR,
    AUTO_NEAR
};

class Robot : public frc::TimedRobot {
public:
    frc::SendableChooser<AutoType> autoChooser;
    AutoType m_autoSelected;  

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
