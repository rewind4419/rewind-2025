#pragma once

#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

namespace AutoConstants {
    
    // =============================================================================
    // SCORING POSITIONS AND WAYPOINTS
    // =============================================================================
    
    // Distance from AprilTag for approach waypoint (meters)
    constexpr double APPROACH_DISTANCE = 1.558;
    constexpr double SCORING_DISTANCE = 0.540;
    constexpr double RETREAT_DISTANCE = 1.0;
    
    // Side offsets for left/right tree positioning (meters)
    constexpr double SIDE_OFFSET = 0.156;
    constexpr double SCORING_SIDE_OFFSET = 0.162;
    
    // Height-specific position adjustments
    constexpr double L4_SCORING_OFFSET = -0.04;  // Additional forward offset for L4 scoring
    
    // Human player station positioning
    constexpr double HUMAN_PLAYER_DISTANCE = 0.5;
    constexpr double HUMAN_PLAYER_APPROACH_TIME = 2.0;
    constexpr double HUMAN_PLAYER_BACKUP_SPEED = -0.5;
    constexpr double HUMAN_PLAYER_RETREAT_TIME = 0.5;
    constexpr double HUMAN_PLAYER_RETREAT_SPEED = 1.0;
    
    // =============================================================================
    // MOVEMENT PARAMETERS
    // =============================================================================
    
    // Swerve drive speeds and timeouts
    constexpr double DEFAULT_APPROACH_SPEED = 3.0;
    constexpr double DEFAULT_SCORING_SPEED = 1.5;
    constexpr double HUMAN_PLAYER_APPROACH_SPEED = 4.0;
    
    // Position and velocity tolerances
    constexpr double APPROACH_POSITION_TOLERANCE = 0.15;
    constexpr double APPROACH_VELOCITY_TOLERANCE = 0.03;
    constexpr double SCORING_POSITION_TOLERANCE = 0.05;
    constexpr double SCORING_VELOCITY_TOLERANCE = 0.03;
    constexpr double HUMAN_PLAYER_POSITION_TOLERANCE = 0.08;
    constexpr double HUMAN_PLAYER_VELOCITY_TOLERANCE = 0.05;
    
    // Maximum velocities and accelerations
    constexpr double MAX_APPROACH_VELOCITY = 3.5;
    constexpr double MAX_APPROACH_ACCELERATION = 2.0;
    constexpr double MAX_SCORING_VELOCITY = 3.5;
    constexpr double MAX_SCORING_ACCELERATION = 3.0;
    constexpr double RETREAT_VELOCITY = 3.5;
    constexpr double RETREAT_ACCELERATION = 2.0;
    constexpr double HUMAN_PLAYER_MAX_VELOCITY = 4.0;
    constexpr double HUMAN_PLAYER_MAX_ACCELERATION = 2.0;
    
    // Emergency/backup drive parameters
    constexpr double EMERGENCY_DRIVE_TIME = 3.0;
    constexpr double EMERGENCY_DRIVE_SPEED = 0.5;
    constexpr double EMERGENCY_TROUGH_DRIVE_TIME = 6.0;
    constexpr double EMERGENCY_TROUGH_DRIVE_SPEED = 1.0;
    constexpr double EMERGENCY_BACKUP_TIME = 1.0;
    constexpr double EMERGENCY_BACKUP_SPEED = -1.0;
    constexpr double BACKUP_SEQUENCE_TIME = 1.5;
    constexpr double BACKUP_SEQUENCE_SPEED = -1.0;
    
    // =============================================================================
    // TIMING PARAMETERS
    // =============================================================================
    
    // Intake and outtake timing
    constexpr double INTAKE_DURATION = 0.2;
    constexpr double OUTTAKE_DURATION = 0.3;
    constexpr double SCORE_DELAY = 0.2;  // Wait before scoring (currently commented out)
    constexpr double SCORE_OUTTAKE_DELAY = 0.3;  // Wait after starting outtake before retreat
    constexpr double TROUGH_SCORE_DELAY = 2.0;  // Outtake duration for trough scoring
    
    // =============================================================================
    // WRIST POSITIONS (MECHANISM-SPECIFIC)
    // =============================================================================
    
    // Special wrist positions for auto scoring
    constexpr double AUTO_SCORING_WRIST_POSITION = 0.436;  // General auto scoring position
    
    // =============================================================================
    // APRILTAG IDs BY ALLIANCE AND POSITION
    // =============================================================================
    
    namespace RedAlliance {
        // Reef tags (piece 1 scoring)
        constexpr int REEF_LEFT = 11;
        constexpr int REEF_CENTER = 10;
        constexpr int REEF_RIGHT = 9;
        
        // Piece 2 scoring tags
        constexpr int PIECE2_FRONT = 7;
        constexpr int PIECE2_MIDDLE_FROM_LEFT = 6;   // When piece 1 was on left
        constexpr int PIECE2_MIDDLE_FROM_RIGHT = 8;  // When piece 1 was on right
        constexpr int PIECE2_FAR_FROM_LEFT = 11;     // When piece 1 was on left
        constexpr int PIECE2_FAR_FROM_RIGHT = 9;     // When piece 1 was on right
        
        // Human player station tags
        constexpr int HUMAN_PLAYER_LEFT_SIDE = 1;   // When starting from left
        constexpr int HUMAN_PLAYER_RIGHT_SIDE = 2;  // When starting from right
    }
    
    namespace BlueAlliance {
        // Reef tags (piece 1 scoring)
        constexpr int REEF_LEFT = 20;
        constexpr int REEF_CENTER = 21;
        constexpr int REEF_RIGHT = 22;
        
        // Piece 2 scoring tags  
        constexpr int PIECE2_FRONT = 18;
        constexpr int PIECE2_MIDDLE_FROM_LEFT = 19;   // When piece 1 was on left
        constexpr int PIECE2_MIDDLE_FROM_RIGHT = 17;  // When piece 1 was on right
        constexpr int PIECE2_FAR_FROM_LEFT = 20;      // When piece 1 was on left
        constexpr int PIECE2_FAR_FROM_RIGHT = 22;     // When piece 1 was on right
        
        // Human player station tags
        constexpr int HUMAN_PLAYER_LEFT_SIDE = 13;   // When starting from left
        constexpr int HUMAN_PLAYER_RIGHT_SIDE = 12;  // When starting from right
    }
    
    // =============================================================================
    // VALIDATION AND SAFETY
    // =============================================================================
    
    // Maximum reasonable timeouts to prevent infinite waits
    constexpr double MAX_WAYPOINT_TIMEOUT = 10.0;
    constexpr double MAX_MECHANISM_TIMEOUT = 5.0;
    constexpr double MAX_SCORING_TIMEOUT = 3.0;
    
} // namespace AutoConstants
