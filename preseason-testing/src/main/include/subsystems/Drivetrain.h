#include <rev/CANSparkMax.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <ctre/phoenix/sensors/CANCoder.h>


class Drivetrain
{
public:
    Drivetrain();

    void SetVec(float x, float y, float r);

    void Update();

    // Swerve module locations
    frc::Translation2d m_frontLeftLocation{0.381_m, 0.381_m};
    frc::Translation2d m_frontRightLocation{0.381_m, -0.381_m};
    frc::Translation2d m_backLeftLocation{-0.381_m, 0.381_m};
    frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

    frc::SwerveDriveKinematics<4> kinematics{
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    };

    ctre::phoenix::sensors::CANCoder flEncoder {12};
    ctre::phoenix::sensors::CANCoder frEncoder {10};
    ctre::phoenix::sensors::CANCoder blEncoder {13};
    ctre::phoenix::sensors::CANCoder brEncoder {11};

    rev::CANSparkMax flDrive {3, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkMax frDrive {8, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkMax blDrive {2, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkMax brDrive {5, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};

    rev::CANSparkMax flSteer {7, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkMax frSteer {4, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkMax blSteer {6, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkMax brSteer {9, rev::CANSparkBase::CANSparkLowLevel::MotorType::kBrushless};
};
