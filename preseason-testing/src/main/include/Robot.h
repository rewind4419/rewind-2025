#pragma once

#include "subsystems/Drivetrain.h"
#include "util/Controller.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/TimedRobot.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc/RobotController.h>

class Robot : public frc::TimedRobot {
public:
    frc::ShuffleboardTab& robotTab = frc::Shuffleboard::GetTab("Robot");

    // the report values of the Robot subsystem
    nt::GenericEntry* selectedAuto = robotTab.AddInteger("Selected Auto", {return 5;});
    nt::GenericEntry* selectedAutoName = robotTab.AddString("Currently selected:", "N/A").GetEntry();
    
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
