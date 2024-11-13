#include <rev/CANSparkMax.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

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
};
