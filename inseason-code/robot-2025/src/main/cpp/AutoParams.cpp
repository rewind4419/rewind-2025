#include "AutoParams.h"
#include <algorithm>
#include <iostream>

void AutoParams::InitializeParameters() {
    // Load parameters from NetworkTables or set defaults
    UpdateFromNetworkTables();
    
    // Publish all parameters to NetworkTables so they're visible on dashboard
    PublishToNetworkTables();
    
    // Validate loaded parameters
    ValidateAndClampParameters();
    
    std::cout << "[AutoParams] Initialized autonomous parameters" << std::endl;
    LogCurrentParameters();
}

void AutoParams::UpdateFromNetworkTables() {
    // Movement parameters
    approach_speed = GetParam("ApproachSpeed", AutoConstants::DEFAULT_APPROACH_SPEED);
    approach_position_tolerance = GetParam("ApproachPosTolerance", AutoConstants::APPROACH_POSITION_TOLERANCE);
    approach_velocity_tolerance = GetParam("ApproachVelTolerance", AutoConstants::APPROACH_VELOCITY_TOLERANCE);
    approach_max_velocity = GetParam("ApproachMaxVel", AutoConstants::MAX_APPROACH_VELOCITY);
    approach_max_acceleration = GetParam("ApproachMaxAccel", AutoConstants::MAX_APPROACH_ACCELERATION);
    
    scoring_speed = GetParam("ScoringSpeed", AutoConstants::DEFAULT_SCORING_SPEED);
    scoring_position_tolerance = GetParam("ScoringPosTolerance", AutoConstants::SCORING_POSITION_TOLERANCE);
    scoring_velocity_tolerance = GetParam("ScoringVelTolerance", AutoConstants::SCORING_VELOCITY_TOLERANCE);
    scoring_max_velocity = GetParam("ScoringMaxVel", AutoConstants::MAX_SCORING_VELOCITY);
    scoring_max_acceleration = GetParam("ScoringMaxAccel", AutoConstants::MAX_SCORING_ACCELERATION);
    
    human_player_speed = GetParam("HumanPlayerSpeed", AutoConstants::HUMAN_PLAYER_APPROACH_SPEED);
    human_player_position_tolerance = GetParam("HumanPlayerPosTolerance", AutoConstants::HUMAN_PLAYER_POSITION_TOLERANCE);
    human_player_velocity_tolerance = GetParam("HumanPlayerVelTolerance", AutoConstants::HUMAN_PLAYER_VELOCITY_TOLERANCE);
    human_player_max_velocity = GetParam("HumanPlayerMaxVel", AutoConstants::HUMAN_PLAYER_MAX_VELOCITY);
    human_player_max_acceleration = GetParam("HumanPlayerMaxAccel", AutoConstants::HUMAN_PLAYER_MAX_ACCELERATION);
    
    // Position adjustments
    approach_distance_offset = GetParam("ApproachDistanceOffset", 0.0);
    scoring_distance_offset = GetParam("ScoringDistanceOffset", 0.0);
    side_offset_adjustment = GetParam("SideOffsetAdjustment", 0.0);
    l4_offset_adjustment = GetParam("L4OffsetAdjustment", 0.0);
    auto_wrist_adjustment = GetParam("AutoWristAdjustment", 0.0);
    
    // Timing parameters
    intake_delay = GetParam("IntakeDelay", AutoConstants::INTAKE_DURATION);
    outtake_delay = GetParam("OuttakeDelay", AutoConstants::OUTTAKE_DURATION);
    score_outtake_delay = GetParam("ScoreOuttakeDelay", AutoConstants::SCORE_OUTTAKE_DELAY);
    mechanism_timeout_multiplier = GetParam("MechanismTimeoutMultiplier", 1.0);
    waypoint_timeout_multiplier = GetParam("WaypointTimeoutMultiplier", 1.0);
    
    // Strategy parameters
    enable_adaptive_timing = GetParam("EnableAdaptiveTiming", false);
    max_retry_attempts = GetParam("MaxRetryAttempts", 1.0);

    // Sensor thresholds
    acquire_current_threshold = GetParam("AcquireCurrentA", 15.0);
    acquire_hold_time = GetParam("AcquireHoldS", 0.06);
    acquire_timeout = GetParam("AcquireTimeoutS", 1.2);
    release_current_drop = GetParam("ReleaseDropA", 8.0);
    release_hold_time = GetParam("ReleaseHoldS", 0.05);
    release_timeout = GetParam("ReleaseTimeoutS", 0.6);
    // Additional timing refinements
    release_presample_time = GetParam("ReleasePresampleS", 0.10);
    l4_settle_time = GetParam("L4SettleS", 0.10);
}

void AutoParams::PublishToNetworkTables() const {
    // Movement parameters
    SetParam("ApproachSpeed", approach_speed);
    SetParam("ApproachPosTolerance", approach_position_tolerance);
    SetParam("ApproachVelTolerance", approach_velocity_tolerance);
    SetParam("ApproachMaxVel", approach_max_velocity);
    SetParam("ApproachMaxAccel", approach_max_acceleration);
    
    SetParam("ScoringSpeed", scoring_speed);
    SetParam("ScoringPosTolerance", scoring_position_tolerance);
    SetParam("ScoringVelTolerance", scoring_velocity_tolerance);
    SetParam("ScoringMaxVel", scoring_max_velocity);
    SetParam("ScoringMaxAccel", scoring_max_acceleration);
    
    SetParam("HumanPlayerSpeed", human_player_speed);
    SetParam("HumanPlayerPosTolerance", human_player_position_tolerance);
    SetParam("HumanPlayerVelTolerance", human_player_velocity_tolerance);
    SetParam("HumanPlayerMaxVel", human_player_max_velocity);
    SetParam("HumanPlayerMaxAccel", human_player_max_acceleration);
    
    // Position adjustments
    SetParam("ApproachDistanceOffset", approach_distance_offset);
    SetParam("ScoringDistanceOffset", scoring_distance_offset);
    SetParam("SideOffsetAdjustment", side_offset_adjustment);
    SetParam("L4OffsetAdjustment", l4_offset_adjustment);
    SetParam("AutoWristAdjustment", auto_wrist_adjustment);
    
    // Timing parameters
    SetParam("IntakeDelay", intake_delay);
    SetParam("OuttakeDelay", outtake_delay);
    SetParam("ScoreOuttakeDelay", score_outtake_delay);
    SetParam("MechanismTimeoutMultiplier", mechanism_timeout_multiplier);
    SetParam("WaypointTimeoutMultiplier", waypoint_timeout_multiplier);
    
    // Strategy parameters
    SetParam("EnableAdaptiveTiming", enable_adaptive_timing);
    SetParam("MaxRetryAttempts", max_retry_attempts);

    // Sensor thresholds
    SetParam("AcquireCurrentA", acquire_current_threshold);
    SetParam("AcquireHoldS", acquire_hold_time);
    SetParam("AcquireTimeoutS", acquire_timeout);
    SetParam("ReleaseDropA", release_current_drop);
    SetParam("ReleaseHoldS", release_hold_time);
    SetParam("ReleaseTimeoutS", release_timeout);
    SetParam("ReleasePresampleS", release_presample_time);
    SetParam("L4SettleS", l4_settle_time);
}

void AutoParams::ResetToDefaults() {
    // Reset all parameters to compiled defaults
    approach_speed = AutoConstants::DEFAULT_APPROACH_SPEED;
    approach_position_tolerance = AutoConstants::APPROACH_POSITION_TOLERANCE;
    approach_velocity_tolerance = AutoConstants::APPROACH_VELOCITY_TOLERANCE;
    approach_max_velocity = AutoConstants::MAX_APPROACH_VELOCITY;
    approach_max_acceleration = AutoConstants::MAX_APPROACH_ACCELERATION;
    
    scoring_speed = AutoConstants::DEFAULT_SCORING_SPEED;
    scoring_position_tolerance = AutoConstants::SCORING_POSITION_TOLERANCE;
    scoring_velocity_tolerance = AutoConstants::SCORING_VELOCITY_TOLERANCE;
    scoring_max_velocity = AutoConstants::MAX_SCORING_VELOCITY;
    scoring_max_acceleration = AutoConstants::MAX_SCORING_ACCELERATION;
    
    human_player_speed = AutoConstants::HUMAN_PLAYER_APPROACH_SPEED;
    human_player_position_tolerance = AutoConstants::HUMAN_PLAYER_POSITION_TOLERANCE;
    human_player_velocity_tolerance = AutoConstants::HUMAN_PLAYER_VELOCITY_TOLERANCE;
    human_player_max_velocity = AutoConstants::HUMAN_PLAYER_MAX_VELOCITY;
    human_player_max_acceleration = AutoConstants::HUMAN_PLAYER_MAX_ACCELERATION;
    
    // Reset adjustments to zero
    approach_distance_offset = 0.0;
    scoring_distance_offset = 0.0;
    side_offset_adjustment = 0.0;
    l4_offset_adjustment = 0.0;
    auto_wrist_adjustment = 0.0;
    
    // Reset timing
    intake_delay = AutoConstants::INTAKE_DURATION;
    outtake_delay = AutoConstants::OUTTAKE_DURATION;
    score_outtake_delay = AutoConstants::SCORE_OUTTAKE_DELAY;
    mechanism_timeout_multiplier = 1.0;
    waypoint_timeout_multiplier = 1.0;
    
    // Reset strategy flags
    enable_adaptive_timing = false;
    max_retry_attempts = 1.0;
    
    // Publish the defaults
    PublishToNetworkTables();
    
    std::cout << "[AutoParams] Reset all parameters to defaults" << std::endl;
}

double AutoParams::GetAdjustedApproachDistance() const {
    return AutoConstants::APPROACH_DISTANCE + approach_distance_offset;
}

double AutoParams::GetAdjustedScoringDistance() const {
    return AutoConstants::SCORING_DISTANCE + scoring_distance_offset;
}

double AutoParams::GetAdjustedSideOffset() const {
    return AutoConstants::SIDE_OFFSET + side_offset_adjustment;
}

double AutoParams::GetAdjustedL4Offset() const {
    return AutoConstants::L4_SCORING_OFFSET + l4_offset_adjustment;
}

double AutoParams::GetAdjustedAutoWristPosition() const {
    return AutoConstants::AUTO_SCORING_WRIST_POSITION + auto_wrist_adjustment;
}

double AutoParams::GetScaledMechanismTimeout(double baseTimeout) const {
    return std::max(0.1, baseTimeout * mechanism_timeout_multiplier);  // Minimum 0.1 seconds
}

double AutoParams::GetScaledWaypointTimeout(double baseTimeout) const {
    return std::max(0.5, baseTimeout * waypoint_timeout_multiplier);   // Minimum 0.5 seconds
}

void AutoParams::ValidateAndClampParameters() {
    // Clamp speed parameters to reasonable ranges
    approach_speed = std::clamp(approach_speed, 0.1, 6.0);
    scoring_speed = std::clamp(scoring_speed, 0.1, 4.0);
    human_player_speed = std::clamp(human_player_speed, 0.1, 6.0);
    
    // Clamp tolerance parameters
    approach_position_tolerance = std::clamp(approach_position_tolerance, 0.01, 1.0);
    approach_velocity_tolerance = std::clamp(approach_velocity_tolerance, 0.01, 0.5);
    scoring_position_tolerance = std::clamp(scoring_position_tolerance, 0.01, 0.5);
    scoring_velocity_tolerance = std::clamp(scoring_velocity_tolerance, 0.01, 0.2);
    
    // Clamp maximum velocities and accelerations
    approach_max_velocity = std::clamp(approach_max_velocity, 1.0, 8.0);
    approach_max_acceleration = std::clamp(approach_max_acceleration, 0.5, 5.0);
    scoring_max_velocity = std::clamp(scoring_max_velocity, 1.0, 6.0);
    scoring_max_acceleration = std::clamp(scoring_max_acceleration, 0.5, 5.0);
    
    // Clamp position offsets to prevent extreme values
    approach_distance_offset = std::clamp(approach_distance_offset, -0.5, 0.5);
    scoring_distance_offset = std::clamp(scoring_distance_offset, -0.3, 0.3);
    side_offset_adjustment = std::clamp(side_offset_adjustment, -0.2, 0.2);
    l4_offset_adjustment = std::clamp(l4_offset_adjustment, -0.1, 0.1);
    auto_wrist_adjustment = std::clamp(auto_wrist_adjustment, -0.1, 0.1);
    
    // Clamp timing parameters
    intake_delay = std::clamp(intake_delay, 0.0, 2.0);
    outtake_delay = std::clamp(outtake_delay, 0.0, 2.0);
    score_outtake_delay = std::clamp(score_outtake_delay, 0.0, 2.0);
    mechanism_timeout_multiplier = std::clamp(mechanism_timeout_multiplier, 0.1, 5.0);
    waypoint_timeout_multiplier = std::clamp(waypoint_timeout_multiplier, 0.1, 5.0);
    
    // Clamp retry attempts
    max_retry_attempts = std::clamp(max_retry_attempts, 0.0, 2.0);

    // Clamp sensor thresholds
    acquire_current_threshold = std::clamp(acquire_current_threshold, 0.0, 80.0);
    acquire_hold_time = std::clamp(acquire_hold_time, 0.0, 2.0);
    acquire_timeout = std::clamp(acquire_timeout, 0.0, 3.0);
    release_current_drop = std::clamp(release_current_drop, 0.0, 40.0);
    release_hold_time = std::clamp(release_hold_time, 0.0, 2.0);
    release_timeout = std::clamp(release_timeout, 0.0, 3.0);
    release_presample_time = std::clamp(release_presample_time, 0.0, 1.0);
    l4_settle_time = std::clamp(l4_settle_time, 0.0, 0.5);
}

void AutoParams::LogCurrentParameters() const {
    std::cout << "[AutoParams] Current parameter values:" << std::endl;
    std::cout << "  Approach Speed: " << approach_speed << std::endl;
    std::cout << "  Scoring Speed: " << scoring_speed << std::endl;
    std::cout << "  Human Player Speed: " << human_player_speed << std::endl;
    std::cout << "  Approach Distance Offset: " << approach_distance_offset << std::endl;
    std::cout << "  Scoring Distance Offset: " << scoring_distance_offset << std::endl;
    std::cout << "  Side Offset Adjustment: " << side_offset_adjustment << std::endl;
    std::cout << "  L4 Offset Adjustment: " << l4_offset_adjustment << std::endl;
    std::cout << "  Auto Wrist Adjustment: " << auto_wrist_adjustment << std::endl;
    std::cout << "  Adaptive Timing: " << (enable_adaptive_timing ? "Enabled" : "Disabled") << std::endl;
    std::cout << "  Max Retry Attempts: " << max_retry_attempts << std::endl;
}
