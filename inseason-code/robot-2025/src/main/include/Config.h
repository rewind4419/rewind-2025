#pragma once

// #define CORAL_INTAKE_MOTOR_1_ID 1
// #define CORAL_INTAKE_MOTOR_2_ID 2
#define CORAL_INTAKE_MOTOR_ID 1
//#define CORAL_INTAKE_MOTOR_ID_2 2

#define CORAL_ARM_MOTOR_ID 5

#define ELEVATOR_MOTOR_1_ID 3
#define ELEVATOR_MOTOR_2_ID 4

#define WINCH_MOTOR_ID 6

#define ALGAE_HANDLER_MOTOR_ID1 7
#define ALGAE_HANDLER_MOTOR_ID2 8

#define INTAKE_MOTOR_ID 9

// TODO: implement all the safeties! only basic ones exist now
#define ELEVATOR_MIN 0_tr
#define ELEVATOR_FUNNEL 1.5_tr // To grab from funnel
#define ELEVATOR_IN_MAX 4.0_tr // Highest elevator can go when coral arm at 0
#define ELEVATOR_SAFE_MAX 18_tr // Highest elevator can go when we are at CORAL_ARM_MIN_SAFE or higher

#define CORAL_ARM_MIN 0_tr
#define CORAL_ARM_INTAKING_MAX 0.025_tr
#define CORAL_ARM_SAFE 0.3_tr // Any less than this, and can't extend all the way
#define CORAL_ARM_MAX 0.40_tr // Arbitrary

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
- Climber
- Algae Handler
*/
