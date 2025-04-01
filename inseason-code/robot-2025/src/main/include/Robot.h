#pragma once

#include <frc/TimedRobot.h>

#include "SwerveConstants.h"
#include "SwerveDrivetrain.h"

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/PS4Controller.h>

#include "queue/Queue.h"
#include "queue/StandardTasks.h"
#include "queue/FrcTasks.h"

#include "util/Controller.h"

#include "Hardware.h"
#include "State.h"
#include "SwervePather.h"
#include "Vision.h"

class Robot : public frc::TimedRobot {
public:
    Queue queue;

    StateManager stateManager;

    SwerveDrivetrain drivetrain {TunerConstants::CreateDrivetrain()};
    Hardware hardware;

    Controller driver {0};
    Controller mate {1};

    SwervePather pather {&drivetrain};

    VisionManager visionManager;

    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
    .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors

    frc::Field2d m_field {};
    frc::FieldObject2d* m_object;

    bool visionEnabled = false;

    Robot();
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void DisabledExit() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void AutonomousExit() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TeleopExit() override;
    void TestInit() override;
    void TestPeriodic() override;
    void TestExit() override;

    // These two functions are in AutoManager.cpp
    void InitializeAutos();
    void RunAuto();

    frc::SendableChooser<int> autoChooser;

    // ctre::phoenix6::hardware::CANcoder fl {8, "Default Name"};
    // ctre::phoenix6::hardware::CANcoder fr {9, "Default Name"};
    // ctre::phoenix6::hardware::CANcoder bl {10, "Default Name"};
    // ctre::phoenix6::hardware::CANcoder br {11, "Default Name"};
};
