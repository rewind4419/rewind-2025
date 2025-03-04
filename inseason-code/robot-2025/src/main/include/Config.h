#pragma once

#define CORAL_INTAKE_MOTOR_1_ID 1
#define CORAL_INTAKE_MOTOR_2_ID 2

#define CORAL_ARM_MOTOR_ID 5

#define ELEVATOR_MOTOR_1_ID 3
#define ELEVATOR_MOTOR_2_ID 4

#define WINCH_MOTOR_ID 6


/*
Notes
- Binding something to a function that is supposed to
  return a CommandPtr but returns nothing causes a bootloop!



TODO List

- Fix the black and red swapped wire!
    - it's on motor ID 4, on the elevator

- Make swervepather not absolutely garbage
    1. Add rotation PID
    2. Add feedforwards
    3. Prototype more advanced control algorithms

Subsystems

- Elevator
- Intake
- Arm
*/
