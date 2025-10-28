#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include "AutoConstants.h"

/**
 * Tunable parameters for autonomous routines.
 * These can be adjusted in real-time via SmartDashboard/NetworkTables
 * for optimization during practice and competition.
 */
class AutoParams {
private:
    // Parameter names for NetworkTables
    static constexpr const char* NT_PREFIX = "Auto/";
    
    // Helper to get/set NT values with defaults
    template<typename T>
    T GetParam(const char* name, T defaultValue) const;
    
    template<typename T>
    void SetParam(const char* name, T value) const;
    
public:
    
    // =============================================================================
    // MOVEMENT TUNING PARAMETERS
    // =============================================================================
    
    // Approach phase tuning
    double approach_speed = AutoConstants::DEFAULT_APPROACH_SPEED;
    double approach_position_tolerance = AutoConstants::APPROACH_POSITION_TOLERANCE;
    double approach_velocity_tolerance = AutoConstants::APPROACH_VELOCITY_TOLERANCE;
    double approach_max_velocity = AutoConstants::MAX_APPROACH_VELOCITY;
    double approach_max_acceleration = AutoConstants::MAX_APPROACH_ACCELERATION;
    
    // Scoring phase tuning  
    double scoring_speed = AutoConstants::DEFAULT_SCORING_SPEED;
    double scoring_position_tolerance = AutoConstants::SCORING_POSITION_TOLERANCE;
    double scoring_velocity_tolerance = AutoConstants::SCORING_VELOCITY_TOLERANCE;
    double scoring_max_velocity = AutoConstants::MAX_SCORING_VELOCITY;
    double scoring_max_acceleration = AutoConstants::MAX_SCORING_ACCELERATION;
    
    // Human player approach tuning
    double human_player_speed = AutoConstants::HUMAN_PLAYER_APPROACH_SPEED;
    double human_player_position_tolerance = AutoConstants::HUMAN_PLAYER_POSITION_TOLERANCE;
    double human_player_velocity_tolerance = AutoConstants::HUMAN_PLAYER_VELOCITY_TOLERANCE;
    double human_player_max_velocity = AutoConstants::HUMAN_PLAYER_MAX_VELOCITY;
    double human_player_max_acceleration = AutoConstants::HUMAN_PLAYER_MAX_ACCELERATION;
    
    // =============================================================================
    // POSITION TUNING PARAMETERS  
    // =============================================================================
    
    // Distance adjustments
    double approach_distance_offset = 0.0;  // Adjustment to default approach distance
    double scoring_distance_offset = 0.0;   // Adjustment to default scoring distance
    double side_offset_adjustment = 0.0;    // Adjustment to left/right positioning
    
    // Height-specific adjustments
    double l4_offset_adjustment = 0.0;      // Additional adjustment for L4 scoring offset
    
    // Wrist position fine-tuning
    double auto_wrist_adjustment = 0.0;     // Adjustment to auto scoring wrist position
    
    // =============================================================================
    // TIMING TUNING PARAMETERS
    // =============================================================================
    
    // Intake/outtake timing
    double intake_delay = AutoConstants::INTAKE_DURATION;
    double outtake_delay = AutoConstants::OUTTAKE_DURATION;
    double score_outtake_delay = AutoConstants::SCORE_OUTTAKE_DELAY;
    
    // Mechanism movement timing
    double mechanism_timeout_multiplier = 1.0;  // Scale all mechanism timeouts
    double waypoint_timeout_multiplier = 1.0;   // Scale all waypoint timeouts
    
    // =============================================================================
    // STRATEGY PARAMETERS
    // =============================================================================
    
    // Performance vs reliability tradeoffs
    bool enable_adaptive_timing = false;        // Use mechanism feedback instead of fixed delays
    double max_retry_attempts = 1.0;            // Number of times to retry failed actions (e.g., jiggle)
    
    // Sensor-based acquire/release thresholds
    double acquire_current_threshold = 15.0;   // Amps at intake indicating coral captured
    double acquire_hold_time = 0.06;           // Seconds current must exceed threshold
    double acquire_timeout = 1.2;              // Seconds to give up waiting for acquire

    double release_current_drop = 8.0;         // Amps drop from baseline indicating release
    double release_hold_time = 0.05;           // Seconds drop must persist
    double release_timeout = 0.6;              // Seconds to give up waiting for release

    // Additional timing refinements
    double release_presample_time = 0.10;      // Seconds to sample outtake current before detection
    double l4_settle_time = 0.10;              // Seconds to settle at L4 before outtake

    // =============================================================================
    // PUBLIC INTERFACE
    // =============================================================================
    
    /**
     * Initialize parameters from NetworkTables or set defaults.
     * Call this once in Robot constructor.
     */
    void InitializeParameters();
    
    /**
     * Update parameters from NetworkTables.
     * Call this periodically (e.g., in DisabledPeriodic) to allow real-time tuning.
     */
    void UpdateFromNetworkTables();
    
    /**
     * Push current parameter values to NetworkTables.
     * Call this when you want to save current settings as defaults.
     */
    void PublishToNetworkTables() const;
    
    /**
     * Reset all parameters to compiled defaults.
     */
    void ResetToDefaults();
    
    /**
     * Get adjusted distance values that incorporate tuning offsets.
     */
    double GetAdjustedApproachDistance() const;
    double GetAdjustedScoringDistance() const;
    double GetAdjustedSideOffset() const;
    double GetAdjustedL4Offset() const;
    double GetAdjustedAutoWristPosition() const;
    
    /**
     * Get timeout values scaled by multipliers.
     */
    double GetScaledMechanismTimeout(double baseTimeout) const;
    double GetScaledWaypointTimeout(double baseTimeout) const;
    
    /**
     * Validate parameter ranges and fix any out-of-bounds values.
     */
    void ValidateAndClampParameters();
    
    /**
     * Log current parameter values for debugging.
     */
    void LogCurrentParameters() const;
};

// Template implementations
template<typename T>
T AutoParams::GetParam(const char* name, T defaultValue) const {
    std::string fullName = std::string(NT_PREFIX) + name;
    
    if constexpr (std::is_same_v<T, double>) {
        return frc::SmartDashboard::GetNumber(fullName, defaultValue);
    } else if constexpr (std::is_same_v<T, bool>) {
        return frc::SmartDashboard::GetBoolean(fullName, defaultValue);
    } else if constexpr (std::is_same_v<T, std::string>) {
        return frc::SmartDashboard::GetString(fullName, defaultValue);
    }
    
    return defaultValue;  // Fallback
}

template<typename T>
void AutoParams::SetParam(const char* name, T value) const {
    std::string fullName = std::string(NT_PREFIX) + name;
    
    if constexpr (std::is_same_v<T, double>) {
        frc::SmartDashboard::PutNumber(fullName, value);
    } else if constexpr (std::is_same_v<T, bool>) {
        frc::SmartDashboard::PutBoolean(fullName, value);
    } else if constexpr (std::is_same_v<T, std::string>) {
        frc::SmartDashboard::PutString(fullName, value);
    }
}
