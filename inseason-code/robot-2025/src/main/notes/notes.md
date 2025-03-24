# Notes

## Guide

- "- " to make a bullet point
    - Bullet points after indents become subtasks, signified by an empty dot instead of a filled dot
- [Brackets] for assigning people to a task
- ***\*\*\*3 asterisks on both sides\*\*\**** to bolden and italicize for in progress tasks
- #\#\# Hashtags followed by a space for headers: 1 for main header, 2 for sub header, 3 for mini header
1. "1.", "2." and so on for ordered lists
- \~\~Two tildes on both sides\~\~ for strikethrough texts
- Bullet points BEFORE strikethrough text, otherwise the indentation breaks
    - Example: These 2 are on different lines in the .md, but show as one line due to improper bullet pointing
        - ~~Correct~~
        ~~- Incorrect~~
        - Note how the incorrect one has the "- " in the strikethroughed text instead of outside it

## Reminders

- Binding something to a function that is supposed to
  return a CommandPtr but returns nothing causes a bootloop!

- Queueing a new command will cancel all the currently running commands,
  unless the currently running commands are configured to not be interruptable!
  to configure them, go to the place where the CommandPtr is created, and do
  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
  before returning it.

## TODO List

- IMPORTANT (after SDR To-do List):
    - Fix vision!
        - Especially for Auto!
    - Fix code to exit Recovery Mode
        - Find new button to exit Recovery Mode or Modify Circle code to exit Recovery Mode properly
    - WINCH
        - Create code so that the robot stays still after it climbed while it is enable

- Notes cleanup and maintainment [Everyone] ***(CONTINUOUS)***
    - ~~Create bindings.md for controller bindings~~
    - Update bindings.md whenever controls are changed [Everyone] ***(CONTINUOUS)***
        

- Make swervepather not absolutely garbage [Sherwin] ***(IN PROGRESS)***
    - ~~ Tune closed loop task to go at the target velocity, especially at low velocities~~
    - ~~ Add point following with tasks that end after reaching goal or stalling~~
    - ~~ Test navigation between multiple points with tasks~~
    - Test larger scale waypoint nav, esp once camera is good angle
    - Further tune PIDs
    - Integrate vision

- Wrist code
    - ~~Switch wrist to use a position PID for everything~~
    - Set zero on init using the rev encoder (Waiting for the REV encoder to get wired)

- Full driver code [Sherwin] ***(IN PROGRESS)***
    - ~~Fix chassis control sensitivity~~
    - Find out what controls we need
    - Set heights

- ~~PhotonVision integration [Sam]~~
    - Get new camera mount at 15 degrees
        - Change rotation value in auto.h
    - ~~Find out what is needed for pather~~
        - ~~Figure out how to access the Pose3d from the returned EstimatedRobotPose~~
    - ~~Clean up autonomousperiodic()~~
        1. ~~Make all of it a function~~
        2. ~~Make auto a subsystem~~
    - ~~Measure camera rotation in rads~~
    - ~~Tell the odometry whats its new pose is~~
    

- Wrist [Nethra]
    - Find positions for wrist (not enough time since the robot was preoccupied most of the time for changing the winch)
    - Add positions to the buttons (not enough time since the robot was preoccupied most of the time for changing the winch)
        - ~~(look at how its done in the CoralArm and Elevator and do the same for the coralwrist)~~
    - ~~Joystick control using SetPositionProvider()~~

Robot Efficiency [Nethra]
    - controller up/down button does not move elevator/arm after pressing triangle

## Subsystems

- Elevator
- Intake
- Arm
- Climber
- Algae Handler

## Notes during SDR

### Backlog

- Separate things like scoring into a separate task that can be added in chunks
- Make tasks not built on auto run, instead on robot init
    - See if maybe thats why auto is slow to start

- Make the rotation motion profiling more robust

### Current Tests

- One Piece Auto Far Left needs to be tested, it was made using positions from calibration

- Blue Side Flipping (done inside Swerve Pather) needs testing

- One Piece Close Right is modified from our testing
- 2 Piece CLose Right L2 L3 is the one from our testing, but its way too long
