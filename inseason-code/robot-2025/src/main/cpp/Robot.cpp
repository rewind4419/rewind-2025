#include "Robot.h"

#include "Config.h"

#include "util/maths.h"

#include "Hardware.h"
#include "State.h"

// :pink_bow:


Robot::Robot() {
    
    frc::SmartDashboard::PutData("Field", &m_field);
    m_object = m_field.GetObject("bob");

    frc::SmartDashboard::PutNumber("Elevator SetPos", 0.0);
    
    InitializeAutos();

    visionManager.Init();

    frc::SmartDashboard::PutNumber("Test Arm", 0.35);
    frc::SmartDashboard::PutNumber("Test Wrist", 0.3);
}

void Robot::RobotPeriodic() {
    frc::SmartDashboard::PutNumber("Wrist Pos", hardware.wrist->motor.GetPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Elevator Pos", hardware.elevator->motor.GetPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Arm Pos", hardware.arm->motor.GetPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("Pulley Pos", hardware.pulley->motor.GetPosition().GetValueAsDouble());

    frc::SmartDashboard::PutNumber("CurrentState", stateManager.currentState);
    frc::SmartDashboard::PutNumber("TargetState", stateManager.targetState);

    frc::SmartDashboard::PutNumber("Queue Tasks", queue.TaskCount());

    m_field.SetRobotPose(drivetrain.GetState().Pose);
}

void Robot::TeleopInit() {
    queue.Clear();
    // hardware.elevator->SetTargetPosition(ELEVATOR_MIN);
    // hardware.arm->SetTargetPosition(CORAL_ARM_MIN);
    // hardware.wrist->SetTargetPosition(CORAL_WRIST_MIN);

    if (stateManager.currentState == STATE_DELIVER || stateManager.targetState == STATE_DELIVER)
    {
        if (hardware.elevator->targetPosition > 1)
        {
            hardware.elevator->SetTargetPosition(ELEVATOR_DELIVER_LOW);
            stateManager.height = DeliverHeight::HEIGHT_L2;
        }
    }

    hardware.winch->SetTargetPosition(hardware.winch->motor.GetPosition().GetValueAsDouble());
    hardware.intake->SetTargetVelocity(0.0);
    // drivetrain.GetPigeon2().SetYaw(0_deg,1_s);

    frc::SmartDashboard::PutNumber("Manual Height", ELEVATOR_MIN);
    frc::SmartDashboard::PutNumber("Manual Arm", CORAL_ARM_MIN);
    frc::SmartDashboard::PutNumber("Manual Wrist", CORAL_WRIST_MIN);
}

void Robot::TeleopPeriodic() {
    visionManager.updRoutine();
    
    // if (visionManager.optional.has_value()){
    //     auto autoPose = visionManager.optional.value().estimatedPose.ToPose2d();
    //     //drivetrain.SamplePoseAt(visionManager.optional.value().timestamp).value().Rotation()
    //     auto visionPose = frc::Pose2d(autoPose.X(),autoPose.Y(), drivetrain.GetState().Pose.Rotation());
    //     // printf("Yes, going to %f, %f, %f\n",visionPose.X().value(), visionPose.Y().value(),visionPose.Rotation().Radians().value());
    //     drivetrain.ResetPose(visionPose);
    //     // drivetrain.AddVisionMeasurement(visionPose, visionManager.optional.value().timestamp);
    // }

    if (!driver.GetR1Button())
    {
        drivetrain.SetControl(
            drive
            .WithVelocityX(-deadzone(driver.GetLeftY(), 0.1) * 5.7_mps * 2)
            .WithVelocityY(-deadzone(driver.GetLeftX(), 0.1) * 5.7_mps * 2)
            .WithRotationalRate(-deadzone(driver.GetRightX(), 0.1) * 1.00_rad_per_s * 4) //(was 0.75_rad_per_s)
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

    if (driver.GetCrossButtonPressed()){
        drivetrain.ResetRotation(0_rad);
        // drivetrain.GetPigeon2().SetYaw(0_deg,1_s);
    }
    if (driver.GetTriangleButtonPressed()){
        drivetrain.ResetRotation(M_PI * 1.0_rad);
        // drivetrain.GetPigeon2().SetYaw(180_deg,1_s);
    }
    if (driver.GetSquareButtonPressed()){
        drivetrain.ResetRotation(-0.5*M_PI * 1.0_rad);
        // drivetrain.GetPigeon2().SetYaw(-90_deg,1_s);
    }
    if (driver.GetCircleButtonPressed()){
        drivetrain.ResetRotation(0.5*M_PI * 1.0_rad); 
        // drivetrain.GetPigeon2().SetYaw(90_deg,1_s);
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
        hardware.pulley->SetMode(CTRL_PID_POSITION);
    }

    if (stateManager.currentState != STATE_ALGAE)
    {
        hardware.intake->SetTargetVelocity(CORAL_ARM_INTAKE_SPEED * deadzone(mate.GetR2Axis() * 0.5 + 0.5, 0.1) + CORAL_ARM_OUTTAKE_SPEED * deadzone(mate.GetL2Axis() * 0.5 + 0.5, 0.1));
    }
    else
    {
        hardware.intake->SetTargetVelocity(CORAL_ARM_INTAKE_SPEED_ALGAE * deadzone(mate.GetR2Axis() * 0.5 + 0.5, 0.1) + CORAL_ARM_OUTTAKE_SPEED_ALGAE * deadzone(mate.GetL2Axis() * 0.5 + 0.5, 0.1));
    }

    if (mate.GetCircleButtonPressed())
    {
        if (stateManager.targetState == STATE_DELIVER || stateManager.targetState == STATE_ALGAE)
        {
            printf("Pressing circle, going to deliver\n");
            queue.AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_MIN, true, hardware.wristDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, true, hardware.elevatorDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN, true, hardware.armDefaultEpsilon));
            queue.AddTask(new CurrentStateTask(STATE_NEUTRAL, &stateManager));
        }
        else if (stateManager.targetState == STATE_FUNNEL)
        {
            queue.AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.pulley, FLIPPER_EXTENDED))
            queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN, true, hardware.armDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_MIN, true, hardware.elevatorDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_MIN, true, hardware.wristDefaultEpsilon));
            queue.AddTask(new CurrentStateTask(STATE_NEUTRAL, &stateManager));
        }
        else if (stateManager.targetState == STATE_CLIMB)
        {
            queue.AddTask(new TargetStateTask(STATE_NEUTRAL, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.pulley, FLIPPER_EXTENDED));
            queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_MIN));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_MIN));
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
            // queue.AddTask(new MotorPositionTask(hardware.pulley, FLIPPER_INTAKE, false));
            queue.AddTask(new CurrentStateTask(STATE_DELIVER, &stateManager));
        }
    }

    if (stateManager.targetState == STATE_DELIVER && mate.GetDPadUpPressed())
    {
        stateManager.GoToDeliverHeight(&queue, hardware.elevator, hardware.wrist, hardware.arm, stateManager.GetNextHeight());
        stateManager.SetArmToDeliver(&queue, hardware.arm);
    }

    if (stateManager.targetState == STATE_DELIVER && mate.GetDPadDownPressed())
    {
        stateManager.GoToDeliverHeight(&queue, hardware.elevator, hardware.wrist, hardware.arm, stateManager.GetPreviousHeight());
        stateManager.SetArmToDeliver(&queue, hardware.arm);
    }

    if ((stateManager.targetState == STATE_DELIVER || stateManager.targetState == STATE_ALGAE) && mate.GetDPadLeftPressed())
    {
        // Low Algae
        queue.AddTask(new TargetStateTask(STATE_ALGAE, &stateManager));
        queue.AddTask(new CustomTask([this] {stateManager.algaeHeight = ALGAE_LOW; return true;}));

        // queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_TRANSIT, true, hardware.armDefaultEpsilon));
        queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_ALGAE, false));
        queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_ALGAE_LOW, true, hardware.elevatorDefaultEpsilon));
        queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_ALGAE, true, hardware.armDefaultEpsilon));
        
        queue.AddTask(new CurrentStateTask(STATE_ALGAE, &stateManager));
    }

    if ((stateManager.targetState == STATE_DELIVER || stateManager.targetState == STATE_ALGAE) && mate.GetDPadRightPressed())
    {
        // High Algae
        queue.AddTask(new TargetStateTask(STATE_ALGAE, &stateManager));
        queue.AddTask(new CustomTask([this] {stateManager.algaeHeight = ALGAE_HIGH; return true;}));


        // queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_TRANSIT, true, hardware.armDefaultEpsilon));
        queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_ALGAE, false));
        queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_ALGAE_LOW, true, hardware.elevatorDefaultEpsilon));
        queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_ALGAE_HIGH, true, hardware.elevatorDefaultEpsilon));
        queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_ALGAE, true, hardware.armDefaultEpsilon));

        queue.AddTask(new CurrentStateTask(STATE_ALGAE, &stateManager));
    }

    if (stateManager.targetState == STATE_ALGAE)
    {
        if (mate.GetDPadUpPressed())
        {
            if (stateManager.algaeHeight == ALGAE_LOW)
            {
                queue.AddTask(new TargetStateTask(STATE_DELIVER, &stateManager));
                queue.AddTask(new CustomTask([this] {stateManager.height = HEIGHT_L3; return true;}));
                stateManager.GoToDeliverHeight(&queue, hardware.elevator, hardware.wrist, hardware.arm, HEIGHT_L3);
                stateManager.SetArmToDeliver(&queue, hardware.arm);
                queue.AddTask(new CurrentStateTask(STATE_DELIVER, &stateManager));
            }
            else if (stateManager.algaeHeight == ALGAE_HIGH)
            {
                queue.AddTask(new TargetStateTask(STATE_DELIVER, &stateManager));
                queue.AddTask(new CustomTask([this] {stateManager.height = HEIGHT_L3; return true;}));
                stateManager.GoToDeliverHeight(&queue, hardware.elevator, hardware.wrist, hardware.arm, HEIGHT_L4);
                stateManager.SetArmToDeliver(&queue, hardware.arm);
                queue.AddTask(new CurrentStateTask(STATE_DELIVER, &stateManager));
            }
        }

        if (mate.GetDPadDownPressed())
        {
            if (stateManager.algaeHeight == ALGAE_LOW)
            {
                queue.AddTask(new TargetStateTask(STATE_DELIVER, &stateManager));
                queue.AddTask(new CustomTask([this] {stateManager.height = HEIGHT_L2; return true;}));
                stateManager.GoToDeliverHeight(&queue, hardware.elevator, hardware.wrist, hardware.arm, HEIGHT_L2);
                stateManager.SetArmToDeliver(&queue, hardware.arm);
                queue.AddTask(new CurrentStateTask(STATE_DELIVER, &stateManager));
            }
            else if (stateManager.algaeHeight == ALGAE_HIGH)
            {
                queue.AddTask(new TargetStateTask(STATE_DELIVER, &stateManager));
                queue.AddTask(new CustomTask([this] {stateManager.height = HEIGHT_L2; return true;}));
                stateManager.GoToDeliverHeight(&queue, hardware.elevator, hardware.wrist, hardware.arm, HEIGHT_L3);
                stateManager.SetArmToDeliver(&queue, hardware.arm);
                queue.AddTask(new CurrentStateTask(STATE_DELIVER, &stateManager));
            }
        }
    }


    if (stateManager.currentState == STATE_DELIVER && stateManager.targetState == STATE_DELIVER)
    {
        if (stateManager.height != HEIGHT_L4)
        {
            hardware.wrist->SetTargetPosition(clamp(CORAL_WRIST_EXTENDED - mate.GetRightY() * 0.3, CORAL_WRIST_TROUGH, 0.4));
        }
        else
        {
            hardware.wrist->SetTargetPosition(clamp(CORAL_WRIST_EXTENDED - mate.GetRightY() * 0.2, 0.15, 0.4));
        }
    }

    if (stateManager.targetState == STATE_DELIVER && mate.GetShareButtonPressed())
    {
        queue.AddTask(new MotorPositionTask(hardware.elevator, frc::SmartDashboard::GetNumber("Manual Height", ELEVATOR_MIN)));
        queue.AddTask(new MotorPositionTask(hardware.arm, frc::SmartDashboard::GetNumber("Manual Arm", CORAL_ARM_MIN)));
        queue.AddTask(new MotorPositionTask(hardware.wrist, frc::SmartDashboard::GetNumber("Manual Wrist", CORAL_WRIST_MIN)));
    }

    if (stateManager.targetState == STATE_DELIVER && mate.GetCrossButtonPressed())
    {
        queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_PLACE, true, hardware.armDefaultEpsilon));
    }

    if (mate.GetSquareButtonPressed())
    {
        if (stateManager.targetState == STATE_NEUTRAL)
        {
            queue.AddTask(new TargetStateTask(STATE_FUNNEL, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.elevator, ELEVATOR_FUNNEL, true, hardware.elevatorDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_FUNNEL, true, hardware.wristDefaultEpsilon));
            queue.AddTask(new MotorPositionTask(hardware.pulley, FLIPPER_EXTENDED)); //Nethra: will potentially retract the funnel
            queue.AddTask(new CurrentStateTask(STATE_FUNNEL, &stateManager));
        }
    }

    if (mate.GetTouchpadButtonPressed())
    {
        if (stateManager.targetState == STATE_NEUTRAL)
        {
            queue.AddTask(new TargetStateTask(STATE_CLIMB, &stateManager));
            queue.AddTask(new MotorPositionTask(hardware.pulley, FLIPPER_RETRACTED));
            queue.AddTask(new MotorPositionTask(hardware.arm, CORAL_ARM_CLIMB));
            queue.AddTask(new MotorPositionTask(hardware.wrist, CORAL_WRIST_CLIMB));
            queue.AddTask(new CurrentStateTask(STATE_CLIMB, &stateManager));
        }
    }

    if (mate.GetOptionsButtonPressed())
    {
        queue.Clear();
    }

    if (driver.GetDPadUpPressed())
    {
        visionManager.tagsAllowed = ALL;
    }

    if (driver.GetDPadLeftPressed())
    {
        visionManager.tagsAllowed = RED_REEF;
    }

    if (driver.GetDPadRightPressed())
    {
        visionManager.tagsAllowed = BLUE_REEF;
    }
    if (driver.GetL2Button()) //Nethra (for new climber wheels but does not work)
    {
        hardware.wheels->SetMode(CTRL_VOLTAGE);
        hardware.wheels->targetVoltage = 6;
        hardware.wheels->targetPosition = hardware.wheels->motor.GetPosition().GetValueAsDouble();
    }
    if (driver.GetR2Button()) //Nethra (for new climber wheels but does not work)
    {
        hardware.wheels->SetMode(CTRL_VOLTAGE);
        hardware.wheels->targetVoltage = -6;
        hardware.wheels->targetPosition = hardware.wheels->motor.GetPosition().GetValueAsDouble();
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
    RunAuto();
}

void Robot::AutonomousPeriodic() {

    visionManager.updRoutine();
    
    // This code actually sets the drivetrain's pose to the vision pose, so add this to teleop periodic to make vision work in teleop
    if (visionManager.optionalVisionEstimate.has_value()){
        auto autoPose = visionManager.optionalVisionEstimate.value().estimatedPose.ToPose2d();
        //drivetrain.SamplePoseAt(visionManager.optional.value().timestamp).value().Rotation()
        //auto visionPose = frc::Pose2d(autoPose.X(),autoPose.Y(),0.0_rad);
        // printf("Yes, going to %f, %f, %f\n",visionPose.X().value(), visionPose.Y().value(),visionPose.Rotation().Radians().value());
        drivetrain.ResetPose(autoPose);
        // drivetrain.AddVisionMeasurement(visionPose, visionManager.optional.value().timestamp);
    }

    drivetrain.Periodic(true);
    queue.Update();
    hardware.Update();
}

void Robot::AutonomousExit() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
    visionManager.updRoutine();
    
    if (visionManager.optionalVisionEstimate.has_value()){
        auto autoPose = visionManager.optionalVisionEstimate.value().estimatedPose.ToPose2d();
        //drivetrain.SamplePoseAt(visionManager.optional.value().timestamp).value().Rotation()
        auto visionPose = frc::Pose2d(autoPose.X(),autoPose.Y(),0.0_rad);
        // printf("Yes, going to %f, %f, %f\n",visionPose.X().value(), visionPose.Y().value(),visionPose.Rotation().Radians().value());
        drivetrain.ResetPose(autoPose);
        // drivetrain.AddVisionMeasurement(visionPose, visionManager.optional.value().timestamp);
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

