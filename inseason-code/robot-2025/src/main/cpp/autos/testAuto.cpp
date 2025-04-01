#include "Robot.h"

#include "Config.h"

#include "util/maths.h"

#include "Hardware.h"
#include "State.h"


void Robot::yeetAuto(VisionManager* vision)
{
    vision->tagsAllowed = RED_REEF;

    printf("hello is the auto??\n");

    TaskList* deploy = new TaskList();

    deploy->AddTask(new TargetStateTask(STATE_DELIVER, &stateManager));
    deploy->AddTask(new CustomTask([this] {stateManager.height = HEIGHT_ZERO; return true;}));
    deploy->AddTask(new ForkTask(
        new MotorPositionTask(hardware.arm, CORAL_ARM_TRANSIT, true, hardware.armDefaultEpsilon),
        new MotorPositionTask(hardware.wrist, CORAL_WRIST_EXTENDED, true, hardware.wristDefaultEpsilon)
    ));
    deploy->AddTask(new CustomTask([this] {stateManager.height = HEIGHT_L2; return true;}));
    deploy->AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_DELIVER_LOW, true, hardware.elevatorDefaultEpsilon));
    deploy->AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_EXTENDED, true, hardware.armDefaultEpsilon));
    deploy->AddTask(new CurrentStateTask(STATE_DELIVER, &stateManager));

    queue.AddTask(new ForkTask{
        new SwerveWaypointTask(&pather, 
            vision->TagToWorld(frc::Pose2d(1.558_m, 0.156_m,0.0_rad),11)
        ,2.0,0.05,0.03,3.5,3.5),
        deploy
    });

    queue.AddTask(new SwerveWaypointTask(&pather, 
        vision->TagToWorld(frc::Pose2d(0.528_m, 0.156_m,0.0_rad),11)
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
        vision->TagToWorld(frc::Pose2d(1.558_m, 0.156_m,0.0_rad), 11)
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