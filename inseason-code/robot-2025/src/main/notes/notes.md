# Notes
- Binding something to a function that is supposed to
  return a CommandPtr but returns nothing causes a bootloop!



## TODO List

- Fix the black and red swapped wire!
    - it's on motor ID 4, on the elevator

- Make swervepather not absolutely garbage
    1. Add rotation PID
    2. Add feedforwards
    3. Prototype more advanced control algorithms

- Create commands for positioning elevator/arm to specific reef level
    - Seperate button to move elevator and arm to each level
    
## Subsystems

- Elevator
- Intake
- Arm
- Climber
- Algae Handler