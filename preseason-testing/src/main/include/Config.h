#pragma once


// -- Swerve Encoder Offsets --
// To calibrate the swerve module rotations:
// 1. Run the robot in Test mode using the Driver Station
//      1.a (or add drivetrain.printCalibationData(); to robotPeriodic)
// 2. Copy the values that are either printed into the config or added to SmartDashboard below
const float CFG_FL_ENCODER_OFFSET = 0.360840;
const float CFG_FR_ENCODER_OFFSET = 0.121094;
const float CFG_BL_ENCODER_OFFSET = -0.012451;
const float CFG_BR_ENCODER_OFFSET = 0.407227;