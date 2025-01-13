#include <rev/CANSparkMax.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <ctre/phoenix6/CANcoder.hpp>

#include "util/PID.h"
#include "Config.h"

class SwerveModule
{
public:
    ctre::phoenix6::hardware::CANcoder encoder;
    rev::CANSparkMax driveMotor;
    rev::CANSparkMax steerMotor;
    PID steerPID;
    float encoderOffset;

    // SwerveModule(
    //     ctre::phoenix6::hardware::CANcoder encoder, 
    //     rev::CANSparkMax driveMotor, 
    //     rev::CANSparkMax steerMotor, 
    //     PID steerPID, 
    //     float encoderOffset
    // );

    void Update(float steerAngle, float drivePower);
};

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

    SwerveModule fl {
        ctre::phoenix6::hardware::CANcoder{12},
        rev::CANSparkMax{8, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless},
        rev::CANSparkMax{7, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless},
        PID {0.3, 0.0, 0.1},
        CFG_FL_ENCODER_OFFSET
    };

    ctre::phoenix6::hardware::CANcoder flEncoder {12};
    ctre::phoenix6::hardware::CANcoder frEncoder {10};
    ctre::phoenix6::hardware::CANcoder blEncoder {13};
    ctre::phoenix6::hardware::CANcoder brEncoder {11};

    rev::CANSparkMax flDrive {3, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkMax frDrive {8, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkMax blDrive {2, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkMax brDrive {5, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};

    rev::CANSparkMax flSteer {7, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkMax frSteer {4, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkMax blSteer {6, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkMax brSteer {9, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};

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
