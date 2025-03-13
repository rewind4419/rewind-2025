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

- ~~Fix the black and red swapped wire!~~
    - ~~It's on motor ID 4, on the elevator~~

- ~~Create commands for positioning elevator/arm to specific reef level~~
    - ~~Seperate button to move elevator and arm to each level~~

- Make swervepather not absolutely garbage
    - Add rotation PID
    - Add feedforwards
    - Prototype more advanced control algorithms

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