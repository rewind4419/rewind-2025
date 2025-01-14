#include <rev/SparkMax.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <ctre/phoenix6/CANcoder.hpp>

#include "util/PID.h"
#include "Config.h"

// class SwerveModule
// {
// public:
//     ctre::phoenix6::hardware::CANcoder& encoder;
//     rev::spark::SparkMax& driveMotor;
//     rev::spark::SparkMax& steerMotor;
//     PID steerPID;
//     float encoderOffset;

//     SwerveModule(
//         ctre::phoenix6::hardware::CANcoder encoder, 
//         rev::spark::SparkMax driveMotor, 
//         rev::spark::SparkMax steerMotor, 
//         PID steerPID, 
//         float encoderOffset
//     );

//     void Update(float steerAngle, float drivePower);
// };

class Drivetrain
{
public:
    Drivetrain();

    const float yawTurnSpeed = 0.5;
    const float translationSpeed = 1.0;

    void SetVec(float x, float y, float r);

    // Swerve module locations
    frc::Translation2d m_frontLeftLocation{0.381_m, 0.381_m};
    frc::Translation2d m_frontRightLocation{0.381_m, -0.381_m};
    frc::Translation2d m_backLeftLocation{-0.381_m, 0.381_m};
    frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

    frc::SwerveDriveKinematics<4> kinematics{
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    };

    // SwerveModule fl {
    //     ctre::phoenix6::hardware::CANcoder{12},
    //     rev::spark::SparkMax{8, rev::spark::SparkLowLevel::MotorType::kBrushless},
    //     rev::spark::SparkMax{7, rev::spark::SparkLowLevel::MotorType::kBrushless},
    //     PID {0.3, 0.0, 0.1},
    //     CFG_FL_ENCODER_OFFSET
    // };

    ctre::phoenix6::hardware::CANcoder flEncoder {12};
    ctre::phoenix6::hardware::CANcoder frEncoder {10};
    ctre::phoenix6::hardware::CANcoder blEncoder {13};
    ctre::phoenix6::hardware::CANcoder brEncoder {11};

    rev::spark::SparkMax flDrive {3, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax frDrive {8, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax blDrive {2, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax brDrive {5, rev::spark::SparkLowLevel::MotorType::kBrushless};

    rev::spark::SparkMax flSteer {7, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax frSteer {4, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax blSteer {6, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax brSteer {9, rev::spark::SparkLowLevel::MotorType::kBrushless};

    PID flSteerPid = PID (0.3, 0.0, 0.1);
    PID frSteerPid = PID (0.3, 0.0, 0.1);
    PID blSteerPid = PID (0.3, 0.0, 0.1);
    PID brSteerPid = PID (0.3, 0.0, 0.1);
    float flSteerTarget;
    float frSteerTarget;
    float blSteerTarget;
    float brSteerTarget;

    void printCalibationData();
};

float moduloAngle(float angle);
