#include "Robot.h"
#include "queue/SensorTasks.h"

#include "Config.h"
#include "AutoConstants.h"

enum Autos
{
    AUTO_NONE,
    AUTO_CONFIGURABLE,
    AUTO_DRIVE_FORWARD_3S,
    AUTO_DRIVE_FORWARD_TROUGH
};


enum Piece1Position
{
    TAG_LEFT,
    TAG_CENTER,
    TAG_RIGHT
};
enum Piece2Position
{
    TAG_FRONT,
    TAG_MIDDLE,
    TAG_FAR
};

enum PiecePlacements
{
    L2_LEFT,
    L2_RIGHT,
    L4_LEFT,
    L4_RIGHT,
    PLACEMENT_NONE
};

enum EndBehavior
{
    GO_TO_CORAL_STATION,
    BACK_UP,
    END_BEHAVIOR_NONE
};

void Robot::InitializeAutos()
{
    autoChooser.SetDefaultOption("No Auto", AUTO_NONE);
    autoChooser.AddOption("ConfigurableAuto", AUTO_CONFIGURABLE);

    autoChooser.AddOption("Auto Drive Forward 3 Seconds", AUTO_DRIVE_FORWARD_3S);
    autoChooser.AddOption("Auto Drive Forward, Trough", AUTO_DRIVE_FORWARD_TROUGH);

    frc::SmartDashboard::PutData(&autoChooser);

    piece1Chooser.SetDefaultOption("Piece 1: L2 Right"      , L2_RIGHT );
    piece1Chooser.AddOption("Piece 1: L2 Left"       , L2_LEFT);
    piece1Chooser.AddOption("Piece 1: L4 Left"       , L4_LEFT);
    piece1Chooser.AddOption("Piece 1: L4 Right"      , L4_RIGHT );
    frc::SmartDashboard::PutData(&piece1Chooser);

    piece2Chooser.SetDefaultOption("Piece 2: L2 Right"      , L2_RIGHT );
    piece2Chooser.AddOption("Piece 2: L2 Left"       , L2_LEFT);
    piece2Chooser.AddOption("Piece 2: L4 Left"       , L4_LEFT);
    piece2Chooser.AddOption("Piece 2: L4 Right"      , L4_RIGHT );
    piece2Chooser.AddOption("Piece 2: None"          , PLACEMENT_NONE );
    frc::SmartDashboard::PutData(&piece2Chooser);

    endChooser.SetDefaultOption("End: Return to coral station", GO_TO_CORAL_STATION);
    endChooser.AddOption("End: Back up for 1.5 seconds", BACK_UP);
    endChooser.AddOption("End: None", END_BEHAVIOR_NONE);
    frc::SmartDashboard::PutData(&endChooser);

    piece1PositionChooser.SetDefaultOption("Piece 1 Side: Center", TAG_CENTER);
    piece1PositionChooser.AddOption("Piece 1 Side: Left", TAG_LEFT);
    piece1PositionChooser.AddOption("Piece 1 Side: Right", TAG_RIGHT);
    frc::SmartDashboard::PutData(&piece1PositionChooser);

    piece2PositionChooser.SetDefaultOption("Piece 2 Side: Middle", TAG_MIDDLE);
    piece2PositionChooser.AddOption("Piece 2 Side: Front", TAG_FRONT);
    piece2PositionChooser.AddOption("Piece 2 Side: Far", TAG_FAR);
    frc::SmartDashboard::PutData(&piece2PositionChooser);
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
    deploy4->AddTask(new MotorPositionTask(hardware.wrist, autoParams.GetAdjustedAutoWristPosition(), true));
    deploy4->AddTask(new CurrentStateTask(STATE_DELIVER, &stateManager));

    double leftTreeMul = 1.0;
    if(scoreOnLeftTree)
    {
        leftTreeMul = -1.0;
    }

    queue.AddTask(new ForkTask{
        new TimedTask(
            new SwerveWaypointTask(&pather, 
                visionManager.TagToWorld(frc::Pose2d(
                    autoParams.GetAdjustedApproachDistance() * 1_m, 
                    leftTreeMul * autoParams.GetAdjustedSideOffset() * 1_m, 
                    0.0_rad
                ), tagId)
            , autoParams.approach_max_velocity
            , autoParams.approach_position_tolerance
            , autoParams.approach_velocity_tolerance
            , autoParams.approach_max_acceleration
            , autoParams.approach_max_velocity),
            autoParams.GetScaledWaypointTimeout(3.0)
        ),
        deploy4
    });

    double L4offset = 0.0;
    if (yourHighness == HEIGHT_L4) {
        L4offset = autoParams.GetAdjustedL4Offset();
    }

    queue.AddTask(new TimedTask(
        new SwerveWaypointTask(&pather, 
            visionManager.TagToWorld(frc::Pose2d(
                (autoParams.GetAdjustedScoringDistance() + L4offset) * 1_m, 
                leftTreeMul * AutoConstants::SCORING_SIDE_OFFSET * 1_m,
                0.0_rad
            ), tagId)
        , autoParams.scoring_max_velocity
        , autoParams.scoring_position_tolerance
        , autoParams.scoring_velocity_tolerance
        , autoParams.scoring_max_acceleration
        , autoParams.scoring_max_velocity),
        autoParams.GetScaledWaypointTimeout(2.5)
    ));

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

    TaskList* releaseThenRetract = new TaskList();
    if (autoParams.enable_adaptive_timing)
    {
        releaseThenRetract->AddTask(new WaitForOuttakeReleaseTask(
            hardware.intake,
            autoParams.release_current_drop,
            autoParams.release_hold_time,
            autoParams.release_timeout
        ));
    }
    else
    {
        releaseThenRetract->AddTask(new DelayTask(autoParams.score_outtake_delay));
    }
    releaseThenRetract->AddTask(new MotorVelocityTask(hardware.intake, 0));
    

    
    if (retractDuringDriveaway)
    {
        releaseThenRetract->AddTask(RetractFromDeliver());
    }
    queue.AddTask(new ForkTask(
        releaseThenRetract,
        new TimedTask(
            new SwerveWaypointTask(&pather, 
                visionManager.TagToWorld(frc::Pose2d(
                    AutoConstants::RETREAT_DISTANCE * 1_m, 
                    leftTreeMul * autoParams.GetAdjustedSideOffset() * 1_m,
                    0.0_rad
                ), tagId)
            , AutoConstants::RETREAT_VELOCITY
            , AutoConstants::APPROACH_POSITION_TOLERANCE
            , 0.1
            , AutoConstants::RETREAT_ACCELERATION
            , AutoConstants::RETREAT_VELOCITY),
            autoParams.GetScaledWaypointTimeout(2.5)
        )
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
                visionManager.TagToWorld(frc::Pose2d(
                    AutoConstants::HUMAN_PLAYER_DISTANCE * 1_m, 
                    0_m, 
                    M_PI * 1_rad
                ), tagId)
            , autoParams.GetScaledWaypointTimeout(autoParams.human_player_speed)
            , autoParams.human_player_position_tolerance
            , autoParams.human_player_velocity_tolerance
            , autoParams.human_player_max_velocity
            , autoParams.human_player_max_acceleration),
            humanPlayerDeploy
        )
    );

    queue.AddTask(
        new SwerveDriveForTask(&pather, 
            AutoConstants::HUMAN_PLAYER_APPROACH_TIME, 
            AutoConstants::HUMAN_PLAYER_BACKUP_SPEED, 
            0.0)
    );

    queue.AddTask(new SwerveLockWheelsTask(&pather));

    queue.AddTask(new MotorVelocityTask(hardware.intake, CORAL_ARM_INTAKE_SPEED));
    if (autoParams.enable_adaptive_timing)
    {
        queue.AddTask(new WaitForIntakeAcquireTask(
            hardware.intake,
            autoParams.acquire_current_threshold,
            autoParams.acquire_hold_time,
            autoParams.acquire_timeout
        ));
    }
    else
    {
        queue.AddTask(new DelayTask(autoParams.intake_delay));
    }

    TaskList* humanPlayerRetract = new TaskList();
    
    humanPlayerRetract->AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
    humanPlayerRetract->AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN, true, hardware.armDefaultEpsilon));
    humanPlayerRetract->AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, true, hardware.elevatorDefaultEpsilon));
    humanPlayerRetract->AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_MIN, true, hardware.wristDefaultEpsilon));
    //humanPlayerRetract->AddTask(new MotorVelocityTask(hardware.intake, 0));
    humanPlayerRetract->AddTask(new CurrentStateTask(STATE_NEUTRAL, &stateManager));

    queue.AddTask(
        new ForkTask(
            new SwerveDriveForTask(&pather, 
                AutoConstants::HUMAN_PLAYER_RETREAT_TIME, 
                AutoConstants::HUMAN_PLAYER_RETREAT_SPEED, 
                0.0),
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

    case AUTO_CONFIGURABLE:
    {
        DeliverHeight piece1Height = HEIGHT_L2;
        DeliverHeight piece2Height = HEIGHT_L2;

        if(piece1Chooser.GetSelected() ==  L4_LEFT || piece1Chooser.GetSelected() ==  L4_RIGHT)
        {
            piece1Height  = HEIGHT_L4;
        }
        if(piece2Chooser.GetSelected() ==  L4_LEFT || piece2Chooser.GetSelected() ==  L4_RIGHT)
        {
            piece2Height  = HEIGHT_L4;
        }
        
        bool piece1Left = piece1Chooser.GetSelected() ==  L2_LEFT || piece1Chooser.GetSelected() ==  L4_LEFT;
        bool piece2Left = piece2Chooser.GetSelected() ==  L2_LEFT || piece2Chooser.GetSelected() ==  L4_LEFT;

        bool doPiece2 = piece2Chooser.GetSelected() != PLACEMENT_NONE && piece1PositionChooser.GetSelected() != TAG_CENTER;

        int tag1;
        int tag2;

        //Select tag 1
        if(isRed){
            switch(piece1PositionChooser.GetSelected())
            {
            case TAG_LEFT:
                    tag1 = 11;
            break;
            
            case TAG_CENTER:
                    tag1 = 10;
            break;
            
            case TAG_RIGHT:
                    tag1 = 9;
            break;

            default: 
                printf("wacky? Piece 1 tag selection is not valid\n");
                return;
            }
        }
        else{
            switch(piece1PositionChooser.GetSelected())
            {
            case TAG_LEFT:
                    tag1 = 20;
            break;
            
            case TAG_CENTER:
                    tag1 = 21;
            break;
            
            case TAG_RIGHT:
                    tag1 = 22;
            break;

            default: 
                printf("wacky? Piece 1 tag selection is not valid\n");
                return;
            }
        }


        //Select tag 2
        if(isRed){
            switch(piece2PositionChooser.GetSelected())
            {
            case TAG_FRONT:
                tag2 = 7;
            break;
            
            case TAG_MIDDLE:
                if(piece1PositionChooser.GetSelected() == TAG_LEFT){
                    tag2 = 6;
                }
                else{
                    tag2 = 8;
                }
            break;
            
            case TAG_FAR:
                if(piece1PositionChooser.GetSelected() == TAG_LEFT){
                    tag2 = 11;
                }
                else{
                    tag2 = 9;
                }
            break;

            default: 
                printf("wacky? Piece 2 tag selection is not valid\n");
                return;
            }
        }
        else{
            switch(piece2PositionChooser.GetSelected())
            {
            case TAG_FRONT:
                tag2 = 18;
            break;
            
            case TAG_MIDDLE:
                if(piece1PositionChooser.GetSelected() == TAG_LEFT){
                    tag2 = 19;
                }
                else{
                    tag2 = 17;
                }
            break;
            
            case TAG_FAR:
                if(piece1PositionChooser.GetSelected() == TAG_LEFT){
                    tag2 = 20;
                }
                else{
                    tag2 = 22;
                }
            break;

            default: 
                printf("wacky? Piece 2 tag selection is not valid\n");
                return;
            }
        }

        int humanPlayerTag;

        if(isRed){
            if(piece1PositionChooser.GetSelected() == TAG_LEFT){
                humanPlayerTag = 1;
            }
            else{
                humanPlayerTag = 2;
            }
        }
        else{
            if(piece1PositionChooser.GetSelected() == TAG_LEFT){
                humanPlayerTag = 13;
            }
            else{
                humanPlayerTag = 12;
            }
        }

        ScoreOnPole(tag1, piece1Height, piece1Left);
        if(doPiece2)
        {
            HumanPlayerPickup(humanPlayerTag);
            ScoreOnPole(tag2, piece2Height, piece2Left);
        }

        int endType = endChooser.GetSelected();
        if(piece1PositionChooser.GetSelected() == TAG_CENTER)
        {
            endType = END_BEHAVIOR_NONE;
        }

        switch (endType)
        {
        case GO_TO_CORAL_STATION:
            HumanPlayerPickup(humanPlayerTag);
            queue.AddTask(new SwerveLockWheelsTask(&pather));
            break;
        case BACK_UP:
            queue.AddTask(new ForkTask(
                new SwerveDriveForTask(&pather,
                    AutoConstants::BACKUP_SEQUENCE_TIME,
                    AutoConstants::BACKUP_SEQUENCE_SPEED,
                    0.0),
                RetractFromDeliver()
            ));
            break;
        case END_BEHAVIOR_NONE:
            queue.AddTask(new SwerveLockWheelsTask(&pather));
            queue.AddTask(RetractFromDeliver());
            break;
        default:
            break;
        }
    }
    break;


    //Emergencies only, no vision
    case AUTO_DRIVE_FORWARD_3S:
    {
        queue.AddTask(new SwerveDriveForTask(&pather, 
            AutoConstants::EMERGENCY_DRIVE_TIME, 
            AutoConstants::EMERGENCY_DRIVE_SPEED, 
            0));
        queue.AddTask(new SwerveLockWheelsTask(&pather));
    }
        break;

    //Emergencies only, no vision
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

        queue.AddTask(new SwerveDriveForTask(&pather, 
            AutoConstants::EMERGENCY_TROUGH_DRIVE_TIME, 
            AutoConstants::EMERGENCY_TROUGH_DRIVE_SPEED, 
            0.0));

        queue.AddTask(new MotorVelocityTask(hardware.intake, CORAL_ARM_OUTTAKE_SPEED));
        queue.AddTask(new DelayTask(AutoConstants::TROUGH_SCORE_DELAY));

        queue.AddTask(new SwerveDriveForTask(&pather, 
            AutoConstants::EMERGENCY_BACKUP_TIME, 
            AutoConstants::EMERGENCY_BACKUP_SPEED, 
            0.0));

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
