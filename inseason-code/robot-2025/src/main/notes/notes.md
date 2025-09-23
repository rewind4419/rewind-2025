# Notes

BRILLIANT WORK CODE TEAM! 2025 AVR auton had a fantastic record of 19 out of 24 coral placed on L4.
Note: appears the resolution for PV might have been set to lower resolution on practice day causing all the autos to fail. All missed were on the LEFT side of the reef perhaps caused by differences in the field elements.

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

## Bugs
- Vision update only runs when disabled (i removed from auto for reasons), implement an updateVision function for auto

## TODO List

- Funnel pulley
    - Create controls, find positions, set position at start of round/auto period, lower funnel during auto (IN PROGRESS - Sam)
- Vision
    - Find new camera position
    - Adjust settings

## Subsystems

- Elevator
- Intake
- Arm
- Climber
- Algae Handler
- Funnel

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
