#pragma once

#define CORAL_INTAKE_MOTOR_ID 1

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
#define ELEVATOR_MIN 0
//#define ELEVATOR_FUNNEL 1.5_tr // Position to grab from funnel
#define ELEVATOR_FUNNEL 0.91 // Position to grab from funnel
#define ELEVATOR_IN_MAX 4.0 // Highest elevator can go when coral arm at 0
#define ELEVATOR_SAFE_MAX 18 // Highest elevator can go when we are at CORAL_ARM_MIN_SAFE or higher

#define ELEVATOR_DELIVER_LOW 3
#define ELEVATOR_DELIVER_MID 8.6
//#define ELEVATOR_DELIVER_HIGH 18
#define ELEVATOR_DELIVER_HIGH 17.7 //was 17.5

#define ELEVATOR_ALGAE_LOW 8.5
#define ELEVATOR_ALGAE_HIGH 14

// just for HIGH, mamke the wrist 0.26 and the arm 0.32

#define CORAL_WRIST_MIN 0
#define CORAL_WRIST_MAX 0.45
#define CORAL_WRIST_INTAKING_MAX 0.025 // All 3 of these are arbitrary at the moment 

#define CORAL_WRIST_EXTENDED 0.32 // was .25 3/30 mh
#define CORAL_WRIST_EXTENDED_L4 0.32 // coral wrist extended, but just for L4
#define CORAL_WRIST_DELIVER_AUTO 0.26 //0.15
#define CORAL_WRIST_DELIVER_AUTO_L4 0.26
#define CORAL_WRIST_CLIMB 0.2
#define CORAL_WRIST_TROUGH 0.075

#define CORAL_WRIST_ALGAE 0.17

//#define CORAL_WRIST_FUNNEL 0.02
#define CORAL_WRIST_FUNNEL 0.0

#define CORAL_ARM_TRANSIT 0.43
#define CORAL_ARM_MIN 0.04444444444 // The angle of the arm when its resting on the hard stop
#define CORAL_ARM_INTAKING_MAX 0.025
#define CORAL_ARM_SAFE 0.3 // Any less than this, and can't extend elevator all the way
#define CORAL_ARM_EXTENDED 0.35
#define CORAL_ARM_MAX 0.44
#define CORAL_ARM_ALGAE 0.43
 // Arbitrary
#define CORAL_ARM_CLIMB 0.20

// Measured in turns per second
#define CORAL_ARM_INTAKE_SPEED 10
#define CORAL_ARM_OUTTAKE_SPEED -12 //-10_tps then -15_tps !NEED TO DEPLOY TO CLOYSTER!
#define CORAL_ARM_INTAKE_SPEED_ALGAE 20
#define CORAL_ARM_OUTTAKE_SPEED_ALGAE -24 //-10_tps then -15_tps !NEED TO DEPLOY TO CLOYSTER!
#define CORAL_ARM_IDLE_SPEED 5

#define FLIPPER_RETRACTED 0
#define FLIPPER_EXTENDED 0.3
