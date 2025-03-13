# Notes
## Guide
__Underlined text__
## Reminders

- Binding something to a function that is supposed to
  return a CommandPtr but returns nothing causes a bootloop!

## TODO List

- Diagnose driver code issue - doing rn
    - For some reason, pressing circle while the elevator moves sets the target state to 0
    and sometimes even moves the arm or the elevator, but then give up half way
    and gets into a garbage state where the target and current states are mismatched

- Fix the black and red swapped wire!
    - it's on motor ID 4, on the elevator

- Create commands for positioning elevator/arm to specific reef level
    - Seperate button to move elevator and arm to each level

- Make swervepather not absolutely garbage
    - Add rotation PID
    - Add feedforwards
    - Prototype more advanced control algorithms

- PhotonVision integration
    - Create kraken-based swerve drive vision code on new bot
    - Find out what is needed for pather ***(DONE)***
        - Figure out how to access the Pose3d from the returned EstimatedRobotPose ***(IN PROGRESS)*** *HEAVY IMPORTANCE*
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