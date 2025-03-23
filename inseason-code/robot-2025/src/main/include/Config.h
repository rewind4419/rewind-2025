#pragma once

// #define CORAL_INTAKE_MOTOR_1_ID 1
// #define CORAL_INTAKE_MOTOR_2_ID 2
#define CORAL_INTAKE_MOTOR_ID 1
//#define CORAL_INTAKE_MOTOR_ID_2 2

#define CORAL_ARM_MOTOR_ID 5
#define CORAL_WRIST_MOTOR_ID 10 // Needs to be set, placeholder

#define ELEVATOR_MOTOR_1_ID 3
#define ELEVATOR_MOTOR_2_ID 4

#define WINCH_MOTOR_ID 6

#define ALGAE_HANDLER_MOTOR_ID1 7
#define ALGAE_HANDLER_MOTOR_ID2 8

#define INTAKE_MOTOR_ID 9

#define FUNNEL_FLIPPER_ID 12

// TODO: implement all the safeties! only basic ones exist now
#define ELEVATOR_MIN 0_tr
//#define ELEVATOR_FUNNEL 1.5_tr // Position to grab from funnel
#define ELEVATOR_FUNNEL 1.0_tr // Position to grab from funnel
#define ELEVATOR_IN_MAX 4.0_tr // Highest elevator can go when coral arm at 0
#define ELEVATOR_SAFE_MAX 18_tr // Highest elevator can go when we are at CORAL_ARM_MIN_SAFE or higher

#define ELEVATOR_DELIVER_LOW 3_tr
#define ELEVATOR_DELIVER_MID 8.6_tr
//#define ELEVATOR_DELIVER_HIGH 18_tr
#define ELEVATOR_DELIVER_HIGH 17.5_tr

// just for HIGH, mamke the wrist 0.26 and the arm 0.32

#define CORAL_WRIST_MIN 0_tr
#define CORAL_WRIST_MAX 0.5_tr
#define CORAL_WRIST_INTAKING_MAX 0.025_tr // All 3 of these are arbitrary at the moment 

#define CORAL_WRIST_EXTENDED 0.3_tr
#define CORAL_WRIST_DELIVER 0.25_tr
#define CORAL_WRIST_CLIMB 0.2_tr

// #define CORAL_WRIST_FUNNEL 0.05_tr
#define CORAL_WRIST_FUNNEL 0.02_tr

#define CORAL_ARM_MIN 0.04444444444_tr // The angle of the arm when its resting on the hard stop
#define CORAL_ARM_INTAKING_MAX 0.025_tr
#define CORAL_ARM_SAFE 0.3_tr // Any less than this, and can't extend elevator all the way
#define CORAL_ARM_EXTENDED 0.35_tr
#define CORAL_ARM_MAX 0.40_tr // Arbitrary
#define CORAL_ARM_CLIMB 0.20_tr

#define CORAL_ARM_INTAKE_SPEED 10_tps
#define CORAL_ARM_OUTTAKE_SPEED -12_tps //-10_tps then -15_tps !NEED TO DEPLOY TO CLOYSTER!
#define CORAL_ARM_IDLE_SPEED 5_tps
