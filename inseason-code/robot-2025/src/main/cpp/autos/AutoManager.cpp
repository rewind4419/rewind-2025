#include "Robot.h"

#include "Config.h"

enum Autos
{
    AUTO_NONE,
    AUTO_DRIVE_FORWARD_3S,
    AUTO_CENTER_L2,
    AUTO_LEFT_L2,
    AUTO_RIGHT_L2,
    AUTO_SCORE_TAG11,
    AUTO_YEETAGE,
    AUTO_DRIVE_FORWARD_TROUGH
};

void Robot::InitializeAutos()
{
    autoChooser.SetDefaultOption("No Auto", AUTO_NONE);
    autoChooser.AddOption("Auto Drive Forward 3 Seconds", AUTO_DRIVE_FORWARD_3S);
    // autoChooser.AddOption("Tag11", AUTO_SCORE_TAG11);
    autoChooser.AddOption("testAuto", AUTO_YEETAGE);

    autoChooser.AddOption("Auto Left L2 (left from driver perspective)", AUTO_LEFT_L2);
    autoChooser.AddOption("Auto Right L2 (right from driver perspective)", AUTO_RIGHT_L2);

    autoChooser.AddOption("Auto Drive Forward, Trough", AUTO_DRIVE_FORWARD_TROUGH);

    frc::SmartDashboard::PutData(&autoChooser);
}

void Robot::RunAuto()
{
    switch (autoChooser.GetSelected())
    {
    case AUTO_NONE:
    {
        queue.AddTask(new CustomTask([] {
            printf("No task\n");
            return true;
        }));
    }
        break;
    case AUTO_YEETAGE:
    {
        visionManager.tagsAllowed = RED_REEF;

        printf("hello is the auto??\n");

        TaskList* deploy4 = new TaskList();

        deploy4->AddTask(new TargetStateTask(STATE_DELIVER, &stateManager));
        deploy4->AddTask(new CustomTask([this] {stateManager.height = HEIGHT_ZERO; return true;}));
        deploy4->AddTask(new ForkTask(
            new MotorPositionTask(hardware.arm, CORAL_ARM_TRANSIT, true, hardware.armDefaultEpsilon),
            new MotorPositionTask(hardware.wrist, CORAL_WRIST_EXTENDED, true, hardware.wristDefaultEpsilon)
        ));
        deploy4->AddTask(new CustomTask([this] {stateManager.height = HEIGHT_L2; return true;}));
        deploy4->AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_DELIVER_LOW, true, hardware.elevatorDefaultEpsilon));
        deploy4->AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_EXTENDED, true, hardware.armDefaultEpsilon));
        deploy4->AddTask(new CurrentStateTask(STATE_DELIVER, &stateManager));

        queue.AddTask(new ForkTask{
            new SwerveWaypointTask(&pather, 
                visionManager.TagToWorld(frc::Pose2d(1.558_m, 0.156_m,0.0_rad),11)
            ,2.0,0.05,0.03,3.5,3.5),
            deploy4
        });

        queue.AddTask(new SwerveWaypointTask(&pather, 
            visionManager.TagToWorld(frc::Pose2d(0.528_m, 0.156_m,0.0_rad),11)
        ,1.5,0.05,0.03,3.5,3.0));
        queue.AddTask(new SwerveLockWheelsTask(&pather));

        queue.AddTask(new DelayTask(0.5));

        queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_DELIVER_AUTO_L4, true, hardware.wristDefaultEpsilon));
        queue.AddTask(new DelayTask(0.3));
        queue.AddTask(new ForkTask(
            new MotorVelocityTask(hardware.intake, CORAL_ARM_OUTTAKE_SPEED),
            new SwerveDriveForTask(&pather, 0.5, -0.5, 0.0)
        ));
        queue.AddTask(new MotorVelocityTask(hardware.intake, 0));
        
        queue.AddTask(new SwerveWaypointTask(&pather, 
            visionManager.TagToWorld(frc::Pose2d(1.558_m, 0.156_m,0.0_rad), 11)
        ,2.0,0.05,0.03,3.5,3.5));

        queue.AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
        queue.AddTask(new ForkTask(
            new MotorPositionTask(hardware.wrist, CORAL_WRIST_MIN, true, hardware.wristDefaultEpsilon),
            new MotorPositionTask(hardware.arm, CORAL_ARM_TRANSIT, true, hardware.armDefaultEpsilon)
        ));
        queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, true, hardware.elevatorDefaultEpsilon));
        queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN, true, hardware.armDefaultEpsilon));
        queue.AddTask(new CurrentStateTask(STATE_NEUTRAL, &stateManager));
    }
        break;

    case AUTO_DRIVE_FORWARD_3S:
    {
        queue.AddTask(new SwerveDriveForTask(&pather, 3.0, 0.5, 0));
        queue.AddTask(new SwerveLockWheelsTask(&pather));
    }
        break;
    case AUTO_DRIVE_FORWARD_TROUGH:
    {
        TaskList* deploy3 = new TaskList();

        deploy3->AddTask(new TargetStateTask(STATE_DELIVER, &stateManager));
        deploy3->AddTask(new CustomTask([this] {stateManager.height = HEIGHT_ZERO; return true;}));
        deploy3->AddTask(new ForkTask(
            new MotorPositionTask(hardware.arm, CORAL_ARM_EXTENDED, true, hardware.armDefaultEpsilon),
            new MotorPositionTask(hardware.wrist, CORAL_WRIST_EXTENDED, true, hardware.wristDefaultEpsilon)
        ));
        deploy3->AddTask(new CustomTask([this] {stateManager.height = HEIGHT_L2; return true;}));
        deploy3->AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_DELIVER_LOW, true, hardware.elevatorDefaultEpsilon));
        deploy3->AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_TROUGH, true, hardware.wristDefaultEpsilon));
        deploy3->AddTask(new CurrentStateTask(STATE_DELIVER, &stateManager));

        queue.AddTask(deploy3);

        queue.AddTask(new SwerveDriveForTask(&pather, 6.0, 1.0, 0.0));

        queue.AddTask(new MotorVelocityTask(hardware.intake, CORAL_ARM_OUTTAKE_SPEED));
        queue.AddTask(new DelayTask(2));

        queue.AddTask(new SwerveDriveForTask(&pather, 1.0, -1.0, 0.0));

        queue.AddTask(new MotorVelocityTask(hardware.intake, 0));

        queue.AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
        queue.AddTask(new ForkTask(
            new MotorPositionTask(hardware.wrist, CORAL_WRIST_MIN, true, hardware.wristDefaultEpsilon),
            new MotorPositionTask(hardware.arm, CORAL_ARM_TRANSIT, true, hardware.armDefaultEpsilon)
        ));
        queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, true, hardware.elevatorDefaultEpsilon));
        queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN, true, hardware.armDefaultEpsilon));
        queue.AddTask(new CurrentStateTask(STATE_NEUTRAL, &stateManager));

        queue.AddTask(new SwerveLockWheelsTask(&pather));
    }
        break;
    }
}
