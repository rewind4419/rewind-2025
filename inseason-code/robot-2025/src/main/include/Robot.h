#pragma once

#include <frc/TimedRobot.h>

#include "SwerveConstants.h"
#include "SwerveDrivetrain.h"

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PS4Controller.h>

#include "queue/Queue.h"
#include "queue/StandardTasks.h"
#include "queue/FrcTasks.h"

#include "Hardware.h"

class Robot : public frc::TimedRobot {
public:
    Queue queue;

    // SwerveDrivetrain drivetrain {TunerConstants::CreateDrivetrain()};
    Hardware hardware;

    frc::PS4Controller driver {0};
    frc::PS4Controller mate {1};




    frc::Field2d m_field {};
    frc::FieldObject2d* m_object;

    

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


    // ctre::phoenix6::hardware::CANcoder fl {8, "Default Name"};
    // ctre::phoenix6::hardware::CANcoder fr {9, "Default Name"};
    // ctre::phoenix6::hardware::CANcoder bl {10, "Default Name"};
    // ctre::phoenix6::hardware::CANcoder br {11, "Default Name"};
};
