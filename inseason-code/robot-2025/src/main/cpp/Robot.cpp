#include "Robot.h"

#include "Config.h"

#include "util/maths.h"

#include "Hardware.h"
#include "State.h"

// :pink_bow:

enum Autos
{
    AUTO_NONE,
    AUTO_DRIVE_FORWARD_1S,
    AUTO_SCORE_TAG11
};

Robot::Robot() {
    
    frc::SmartDashboard::PutData("Field", &m_field);
    m_object = m_field.GetObject("bob");

    frc::SmartDashboard::PutNumber("Elevator SetPos", 0.0);
    
    m.Init();

    autoChooser.SetDefaultOption("No Auto", AUTO_NONE);
    autoChooser.AddOption("Drive Forward 1 Second Auto", AUTO_DRIVE_FORWARD_1S);
    autoChooser.AddOption("Tag11", AUTO_SCORE_TAG11);

    frc::SmartDashboard::PutData(&autoChooser);

    frc::SmartDashboard::PutNumber("Test Arm", 0.35);
    frc::SmartDashboard::PutNumber("Test Wrist", 0.3);
}

void Robot::RobotPeriodic() {
    frc::SmartDashboard::PutNumber("Wrist Pos", hardware.wrist->motor.GetPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Elevator Pos", hardware.elevator->motor.GetPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Arm Pos", hardware.arm->motor.GetPosition().GetValueAsDouble());

    frc::SmartDashboard::PutNumber("CurrentState", stateManager.currentState);
    frc::SmartDashboard::PutNumber("TargetState", stateManager.targetState);
}

void Robot::TeleopInit() {
    queue.Clear();
    hardware.winch->SetTargetPosition(hardware.winch->motor.GetPosition().GetValueAsDouble());
    hardware.intake->SetTargetVelocity(0.0);
}

void Robot::TeleopPeriodic() {

    m.updRoutine();
    if (m.optional.has_value()){
        printf("Yes, going to %f, %f\n", m.optional.value().estimatedPose.X().value(), m.optional.value().estimatedPose.Y().value());
        drivetrain.AddVisionMeasurement(m.optional.value().estimatedPose.ToPose2d(), m.optional.value().timestamp);
    }

    if (!driver.GetR1Button())
    {
        drivetrain.SetControl(
            drive
            .WithVelocityX(-deadzone(driver.GetLeftY(), 0.1) * 5.7_mps * 2)
            .WithVelocityY(-deadzone(driver.GetLeftX(), 0.1) * 5.7_mps * 2)
            .WithRotationalRate(-deadzone(driver.GetRightX(), 0.1) * 0.75_rad_per_s * 4)
        );
    }
    else
    {
        // Slow mode
        drivetrain.SetControl(
            drive
            .WithVelocityX(-deadzone(driver.GetLeftY(), 0.1) * 5.7_mps * 0.35 * 2)
            .WithVelocityY(-deadzone(driver.GetLeftX(), 0.1) * 5.7_mps * 0.35 * 2)
            .WithRotationalRate(-deadzone(driver.GetRightX(), 0.1) * 0.75_rad_per_s * 4 * 0.45)
        );
    }

    if (driver.GetTriangleButtonPressed())
    {
        drivetrain.SeedFieldCentric();
    }

    

    if (mate.GetR1Button())
    {
        hardware.winch->SetMode(CTRL_VOLTAGE);
        hardware.winch->targetVoltage = -6;
        hardware.winch->targetPosition = hardware.winch->motor.GetPosition().GetValueAsDouble();
    }
    else if (mate.GetL1Button())
    {
        hardware.winch->SetMode(CTRL_VOLTAGE);
        hardware.winch->targetVoltage = 6;
        hardware.winch->targetPosition = hardware.winch->motor.GetPosition().GetValueAsDouble();
    }
    else
    {
        hardware.winch->SetMode(CTRL_PID_POSITION);
    }

    hardware.intake->SetTargetVelocity(CORAL_ARM_INTAKE_SPEED * deadzone(mate.GetR2Axis() * 0.5 + 0.5, 0.1) + CORAL_ARM_OUTTAKE_SPEED * deadzone(mate.GetL2Axis() * 0.5 + 0.5, 0.1));

    if (mate.GetCircleButtonPressed())
    {
        if (stateManager.targetState == STATE_DELIVER)
        {
            queue.AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
            queue.AddTask(new ForkTask(
                new MotorPositionTask(hardware.wrist, CORAL_WRIST_FUNNEL, true, hardware.wristDefaultEpsilon),
                new MotorPositionTask(hardware.arm, CORAL_ARM_SAFE, true, hardware.armDefaultEpsilon)
            ));
            queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, true, hardware.elevatorDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN, true, hardware.armDefaultEpsilon));
            queue.AddTask(new CurrentStateTask(STATE_NEUTRAL, &stateManager));
        }
        else if (stateManager.targetState == STATE_FUNNEL)
        {
            queue.AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN, true, hardware.armDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, true, hardware.elevatorDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_FUNNEL, true, hardware.wristDefaultEpsilon));
            queue.AddTask(new CurrentStateTask(STATE_NEUTRAL, &stateManager));
        }
        else if (stateManager.targetState == STATE_CLIMB)
        {
            queue.AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.flipper, FLIPPER_RETRACTED));
            queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_FUNNEL));
            queue.AddTask(new CurrentStateTask(STATE_NEUTRAL, &stateManager));
        }
    }

    if (mate.GetTriangleButtonPressed())
    {
        if (stateManager.targetState == STATE_NEUTRAL)
        {
            queue.AddTask(new TargetStateTask(STATE_DELIVER, &stateManager));
            queue.AddTask(new CustomTask([this] {stateManager.height = HEIGHT_ZERO; return true;}));
            queue.AddTask(new ForkTask(
                new MotorPositionTask(hardware.arm, CORAL_ARM_EXTENDED, true, hardware.armDefaultEpsilon),
                new MotorPositionTask(hardware.wrist, CORAL_WRIST_EXTENDED, true, hardware.wristDefaultEpsilon)
            ));
            queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, false, hardware.elevatorDefaultEpsilon));
            queue.AddTask(new CurrentStateTask(STATE_DELIVER, &stateManager));
        }
    }

    if (stateManager.targetState == STATE_DELIVER && mate.GetDPadUpPressed())
    {
        stateManager.IncrementDeliverHeight();
        stateManager.GoToDeliverHeight(&queue, hardware.elevator, hardware.wrist);
    }

    if (stateManager.targetState == STATE_DELIVER && mate.GetDPadDownPressed())
    {
        stateManager.DecrementDeliverHeight();
        stateManager.GoToDeliverHeight(&queue, hardware.elevator, hardware.wrist);
    }

    // if (stateManager.targetState == STATE_DELIVER && mate.GetDPadRightPressed())
    // {
    //     //queue.AddTask(new MotorPositionTask(hardware.arm, frc::SmartDashboard::GetNumber("Test Arm", 0.35)));
    //     //queue.AddTask(new MotorPositionTask(hardware.wrist, frc::SmartDashboard::GetNumber("Test Wrist", 0.3)));
    // }

    if (stateManager.currentState == STATE_DELIVER && stateManager.targetState == STATE_DELIVER)
    {
        hardware.wrist->SetTargetPosition(CORAL_WRIST_EXTENDED - mate.GetRightY() * 0.1);
    }

    if (mate.GetSquareButtonPressed())
    {
        if (stateManager.targetState == STATE_NEUTRAL)
        {
            queue.AddTask(new TargetStateTask(STATE_FUNNEL, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_FUNNEL, true, hardware.elevatorDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_FUNNEL, true, hardware.wristDefaultEpsilon));
            queue.AddTask(new CurrentStateTask(STATE_FUNNEL, &stateManager));
        }
    }

    if (mate.GetTouchpadButtonPressed())
    {
        if (stateManager.targetState == STATE_NEUTRAL)
        {
            queue.AddTask(new TargetStateTask(STATE_CLIMB, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.flipper, FLIPPER_EXTENDED));
            queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_CLIMB));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_CLIMB));
            queue.AddTask(new CurrentStateTask(STATE_CLIMB, &stateManager));
        }
    }

    drivetrain.Periodic(false);

    queue.Update();
    hardware.Update();

    driver.Update();
    mate.Update();
}

void Robot::TeleopExit() {}

void Robot::AutonomousInit() {
    hardware.intake->SetTargetVelocity(0.0);
    visionEnabled = false;
    queue.Clear();

    

    switch (autoChooser.GetSelected())
    {
    case AUTO_NONE:
        queue.AddTask(new CustomTask([] {
            printf("No task\n");
            return true;
        }));
        break;
    case AUTO_DRIVE_FORWARD_1S:
        TaskList* list = new TaskList();

        list->AddTask(new SwerveDriveForTask(&pather, 1.0, 0.5, 0));
        list->AddTask(new SwerveLockWheelsTask(&pather));
        list->AddTask(new DelayTask(1));
        list->AddTask(new SwerveDriveForTask(&pather, 1.0, -0.5, 0));
        list->AddTask(new SwerveLockWheelsTask(&pather));

        TaskList* list2 = new TaskList();

        list2->AddTask(new ForkTask(
            new MotorPositionTask(hardware.arm, CORAL_ARM_EXTENDED, true, hardware.armDefaultEpsilon),
            new MotorPositionTask(hardware.wrist, CORAL_WRIST_EXTENDED, true, hardware.wristDefaultEpsilon)
        ));
        list2->AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_DELIVER_LOW, true, hardware.elevatorDefaultEpsilon));
        list2->AddTask(new DelayTask(1));
        list2->AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, true, hardware.elevatorDefaultEpsilon));
        list2->AddTask(new ForkTask(
            new MotorPositionTask(hardware.arm, CORAL_ARM_MIN, true, hardware.armDefaultEpsilon),
            new MotorPositionTask(hardware.wrist, CORAL_WRIST_FUNNEL, true, hardware.wristDefaultEpsilon)
        ));

        queue.AddTask(new ForkTask(list, list2));

        break;
    // case AUTO_SCORE_TAG11:
    //     TaskList* tag11List1 = new TaskList();

    //     tag11List1->AddTask(new SwerveWaypointTask(&pather, frc::Pose2d{11.27293_m, 2.46467_m, frc::Rotation2d{1_rad}}, 2, 0.05, 0.1, 2.0, 2.0));
    //     tag11List1->AddTask(new SwerveLockWheelsTask(&pather));

    //     TaskList* tag11List2 = new TaskList();

    //     tag11List2->AddTask(new ForkTask(
    //         new MotorPositionTask(hardware.arm, CORAL_ARM_EXTENDED, true, hardware.armDefaultEpsilon),
    //         new MotorPositionTask(hardware.wrist, CORAL_WRIST_EXTENDED, true, hardware.wristDefaultEpsilon)
    //     ));
    //     tag11List2->AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_DELIVER_LOW, true, hardware.elevatorDefaultEpsilon));

    //     queue.AddTask(new ForkTask(tag11List1, tag11List2));

    //     queue.AddTask(new SwerveWaypointTask(&pather, frc::Pose2d{11.77293_m, 3.3307_m, frc::Rotation2d{1_rad}}, 0.5, 0.05, 0.1, 1.0, 1.0));
    //     queue.AddTask(new SwerveLockWheelsTask(&pather));

    //     break;
    }
}

void Robot::AutonomousPeriodic() {
    if (visionEnabled)
    {
        m.updRoutine();
        if (m.optional.has_value()){
            //printf("Yes, going to %f, %f\n", m.optional.value().estimatedPose.X().value(), m.optional.value().estimatedPose.Y().value());
            drivetrain.AddVisionMeasurement(m.optional.value().estimatedPose.ToPose2d(), m.optional.value().timestamp);
        }
    }

    drivetrain.Periodic(true);
    queue.Update();
    hardware.Update();
}

void Robot::AutonomousExit() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
    m.updRoutine();

    if (m.optional.has_value()){
        //printf("Yes, going to %f, %f\n", m.optional.value().estimatedPose.X().value(), m.optional.value().estimatedPose.Y().value());
        drivetrain.AddVisionMeasurement(m.optional.value().estimatedPose.ToPose2d(), utils::GetCurrentTime());
    }
    drivetrain.Periodic();
}

void Robot::DisabledExit() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {
    // Swerve Calibration
    // frc::SmartDashboard::PutNumber("FL Angle", this->fl.GetAbsolutePosition().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("FR Angle", this->fr.GetAbsolutePosition().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("BL Angle", this->bl.GetAbsolutePosition().GetValueAsDouble());
    // frc::SmartDashboard::PutNumber("BR Angle", this->br.GetAbsolutePosition().GetValueAsDouble());
}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

