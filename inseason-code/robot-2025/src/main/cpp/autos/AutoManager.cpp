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
    AUTO_DRIVE_FORWARD_TROUGH,
    AUTO_2_PIECE_LEFT_L2
};

void Robot::InitializeAutos()
{
    autoChooser.SetDefaultOption("No Auto", AUTO_NONE);
    autoChooser.AddOption("Auto Drive Forward 3 Seconds", AUTO_DRIVE_FORWARD_3S);
    // autoChooser.AddOption("Tag11", AUTO_SCORE_TAG11);
    autoChooser.AddOption("testAuto", AUTO_YEETAGE);

    autoChooser.AddOption("Auto Left L2 (left from driver perspective)", AUTO_LEFT_L2);
    autoChooser.AddOption("Auto Center L2 (center from driver perspective)", AUTO_CENTER_L2);
    autoChooser.AddOption("Auto Right L2 (right from driver perspective)", AUTO_RIGHT_L2);

    autoChooser.AddOption("Auto Drive Forward, Trough", AUTO_DRIVE_FORWARD_TROUGH);

    frc::SmartDashboard::PutData(&autoChooser);
}

TaskList* Robot::RetractFromDeliver()
{
    TaskList* retractList = new TaskList();
    retractList->AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
    retractList->AddTask(new ForkTask(
        new MotorPositionTask(hardware.wrist, CORAL_WRIST_MIN, true, hardware.wristDefaultEpsilon),
        new MotorPositionTask(hardware.arm, CORAL_ARM_TRANSIT, true, hardware.armDefaultEpsilon)
    ));
    retractList->AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, true, hardware.elevatorDefaultEpsilon));
    retractList->AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN, true, hardware.armDefaultEpsilon));
    retractList->AddTask(new CurrentStateTask(STATE_NEUTRAL, &stateManager));
    return retractList;
}

void Robot::ScoreOnPole(int tagId, DeliverHeight yourHighness, bool scoreOnLeftTree, bool retractDuringDriveaway)
{
    TaskList* deploy4 = new TaskList();

    deploy4->AddTask(new TargetStateTask(STATE_DELIVER, &stateManager));
    deploy4->AddTask(new CustomTask([this] {stateManager.height = HEIGHT_ZERO; return true;}));
    deploy4->AddTask(new ForkTask(
        new MotorPositionTask(hardware.arm, CORAL_ARM_TRANSIT, true, hardware.armDefaultEpsilon),
        new MotorPositionTask(hardware.wrist, CORAL_WRIST_EXTENDED, true, hardware.wristDefaultEpsilon)
    ));

    deploy4->AddTask(new CustomTask([this,yourHighness] {stateManager.height = yourHighness; return true;}));

    switch(yourHighness)
    {
    case HEIGHT_L2:
        deploy4->AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_DELIVER_LOW, true, hardware.elevatorDefaultEpsilon));
    break;
    case HEIGHT_L3:
        deploy4->AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_DELIVER_MID, true, hardware.elevatorDefaultEpsilon));
    break;
    case HEIGHT_L4:
        deploy4->AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_DELIVER_HIGH, true, hardware.elevatorDefaultEpsilon));
    break;
    default:
        printf("You can't set the lift to this height in auto!\n");
    break;
    }
    deploy4->AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_EXTENDED, true, hardware.armDefaultEpsilon));
    deploy4->AddTask(new CurrentStateTask(STATE_DELIVER, &stateManager));

    double leftTreeMul = 1.0;
    if(scoreOnLeftTree)
    {
        leftTreeMul = -1.0;
    }

    queue.AddTask(new ForkTask{
        new SwerveWaypointTask(&pather, 
            visionManager.TagToWorld(frc::Pose2d(1.558_m, leftTreeMul * 0.156_m,0.0_rad),tagId)
        ,3.0,0.15,0.03,3.5,2.0),
        deploy4
    });

    queue.AddTask(new SwerveWaypointTask(&pather, 
        visionManager.TagToWorld(frc::Pose2d(0.528_m, leftTreeMul*0.156_m,0.0_rad),tagId)
    ,1.5,0.05,0.03,3.5,3.0));

    queue.AddTask(new SwerveLockWheelsTask(&pather));

    //Slated for destruction
    // queue.AddTask(new DelayTask(0.2));

    if (yourHighness == HEIGHT_L4)
    {
        queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_DELIVER_AUTO_L4, true, hardware.wristDefaultEpsilon));
    }
    else
    {
        queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_PLACE, true, hardware.armDefaultEpsilon));
    }

    //Slated for destruction
    // queue.AddTask(new DelayTask(0.3));
    // queue.AddTask(new ForkTask(
    //     new MotorVelocityTask(hardware.intake, CORAL_ARM_OUTTAKE_SPEED),
    //     new SwerveDriveForTask(&pather, 0.3, -1.0, 0.0)
    // ));
    queue.AddTask(
        new MotorVelocityTask(hardware.intake, CORAL_ARM_OUTTAKE_SPEED)
    );

    TaskList* stopIntaking = new TaskList();
    stopIntaking->AddTask(new DelayTask(0.3));
    stopIntaking->AddTask(new MotorVelocityTask(hardware.intake, 0));
    

    
    if (retractDuringDriveaway)
    {
        stopIntaking->AddTask(RetractFromDeliver());
    }
    queue.AddTask(new ForkTask(
        stopIntaking,
        new SwerveWaypointTask(&pather, 
            visionManager.TagToWorld(frc::Pose2d(1.0_m, leftTreeMul*0.156_m,0.0_rad), tagId)
        ,3.0,0.2,0.1,3.5,2.0)
    ));
    
}

void Robot::HumanPlayerPickup(int tagId)
{
    TaskList* humanPlayerDeploy = RetractFromDeliver();

    humanPlayerDeploy->AddTask(new TargetStateTask(STATE_FUNNEL, &stateManager));
    humanPlayerDeploy->AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_FUNNEL, true, hardware.elevatorDefaultEpsilon));
    humanPlayerDeploy->AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_FUNNEL, true, hardware.wristDefaultEpsilon));
    humanPlayerDeploy->AddTask(new CurrentStateTask(STATE_FUNNEL, &stateManager));
    humanPlayerDeploy->AddTask(new MotorVelocityTask(hardware.intake, CORAL_ARM_OUTTAKE_SPEED));

    queue.AddTask(
        new ForkTask(
            new SwerveWaypointTask(&pather, 
                visionManager.TagToWorld(frc::Pose2d(0.5_m, 0_m, M_PI * 1_rad), tagId)
            ,4.0,0.08,0.05,4.0,2.0),
            humanPlayerDeploy
        )
    );

    queue.AddTask(
        new SwerveDriveForTask(&pather, 2, -0.5, 0.0)
    );

    queue.AddTask(new SwerveLockWheelsTask(&pather));

    queue.AddTask(new MotorVelocityTask(hardware.intake, CORAL_ARM_INTAKE_SPEED));
    queue.AddTask(new DelayTask(0.2));

    TaskList* humanPlayerRetract = new TaskList();
    
    humanPlayerRetract->AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
    humanPlayerRetract->AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN, true, hardware.armDefaultEpsilon));
    humanPlayerRetract->AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, true, hardware.elevatorDefaultEpsilon));
    humanPlayerRetract->AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_MIN, true, hardware.wristDefaultEpsilon));
    //humanPlayerRetract->AddTask(new MotorVelocityTask(hardware.intake, 0));
    humanPlayerRetract->AddTask(new CurrentStateTask(STATE_NEUTRAL, &stateManager));

    queue.AddTask(
        new ForkTask(
            new SwerveDriveForTask(&pather, 0.5, 1.0, 0.0),
            humanPlayerRetract
        )
    );
}

void Robot::RunAuto()
{
    std::optional<frc::DriverStation::Alliance> allianceOptional = frc::DriverStation::GetAlliance();

    bool isRed = true;

    if (allianceOptional.has_value())
    {
        if (allianceOptional.value() == frc::DriverStation::Alliance::kRed)
        {
            visionManager.tagsAllowed = RED_REEF;
            isRed = true;
            printf("Running red\n");
        }
        else
        {
            visionManager.tagsAllowed = BLUE_REEF;
            isRed = false;
            printf("Running blue\n");
        }
    }
    else
    {
        printf("Warning, frc driver station returned no team, aborting auto!!\n");
        return;
    }

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
        ScoreOnPole(20, HEIGHT_L4);
        HumanPlayerPickup(13);
        ScoreOnPole(18, HEIGHT_L4, true);
        HumanPlayerPickup(13);
    }
        break;
    case AUTO_LEFT_L2:
    {
        if (isRed)
        {
            ScoreOnPole(11,HEIGHT_L2);
        }
        else
        {
            ScoreOnPole(20,HEIGHT_L2);
        }
        queue.AddTask(RetractFromDeliver());
        queue.AddTask(new SwerveLockWheelsTask(&pather));
    }
        break;
    case AUTO_RIGHT_L2:
    {
        if (isRed)
        {
            ScoreOnPole(9,HEIGHT_L4);
        }
        else
        {
            ScoreOnPole(22,HEIGHT_L4);
        }
        queue.AddTask(RetractFromDeliver());
        queue.AddTask(new SwerveLockWheelsTask(&pather));
    }
        break;
    case AUTO_CENTER_L2:
    {
        if (isRed)
        {
            ScoreOnPole(10,HEIGHT_L2);
        }
        else
        {
            ScoreOnPole(21,HEIGHT_L2);
        }
        queue.AddTask(RetractFromDeliver());
        queue.AddTask(new SwerveLockWheelsTask(&pather));
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
