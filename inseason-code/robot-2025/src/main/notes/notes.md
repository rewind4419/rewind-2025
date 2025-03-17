# Notes
## Guide

## Reminders

- Binding something to a function that is supposed to
  return a CommandPtr but returns nothing causes a bootloop!

- Queueing a new command will cancel all the currently running commands,
  unless the currently running commands are configured to not be interruptable!
  to configure them, go to the place where the CommandPtr is created, and do
  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
  before returning it.

## TODO List

- Make swervepather not absolutely garbage ***(IN PROGRESS)***
    - Tune closed loop task to go at the target velocity, especially at low velocities
    - Add point following with tasks that end after reaching goal or stalling
    - Test navigation between multiple points with tasks
    - Integrate vision

- Wrist code
    - Switch wrist to use a position PID for everything
    - Set zero on init using the rev encoder

- Full driver code
    - Fix chassis control sensitivity

- PhotonVision integration
    - ~~Find out what is needed for pather~~
        - ~~Figure out how to access the Pose3d from the returned EstimatedRobotPose~~
    - Clean up autonomousperiodic() ***(IN PROGRESS)***
        1. Make all of it a function
        2. Make auto a subsystem
    - Measure camera rotation in rads


- Write an auto code that simply moves the robot forward

- Lower the default position for intaking coral

- Fix driver code
    - Sometimes triangle doesn't work until pressing circle first

## Subsystems

- Elevator
- Intake
- Arm
- Climber
- Algae Handler
