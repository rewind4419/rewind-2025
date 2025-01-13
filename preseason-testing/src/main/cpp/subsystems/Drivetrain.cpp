#include "subsystems/Drivetrain.h"

#include <stdio.h>
#include <math.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/smartdashboard/SmartDashboard.h>

void SwerveModule::Update(float steerAngle, float drivePower)
{
    driveMotor.Set(drivePower);

    float currentAngle = encoder.GetAbsolutePosition().GetValue().value() * 2 * M_PI;

    for () // solve modulo problem without while loop
}

Drivetrain::Drivetrain()
{

}

void Drivetrain::SetVec(float x, float y, float r)
{
    frc::ChassisSpeeds speeds {
        units::meters_per_second_t{x * translationSpeed}, 
        units::meters_per_second_t{y * translationSpeed}, 
        units::radians_per_second_t{r * yawTurnSpeed}
    };

    wpi::array<frc::SwerveModuleState, 4U> states = kinematics.ToSwerveModuleStates(speeds);

    frc::SwerveModuleState fl = states.at(0);
    frc::SwerveModuleState fr = states.at(1);
    frc::SwerveModuleState bl = states.at(2);
    frc::SwerveModuleState br = states.at(3);

    // if (sqrtf(x*x+y*y) > 0.3 || fabsf(r) > 0.1)
    // {
    //     flSteerTarget = fl.angle.Radians().value();
    //     flDrive.Set(fl.speed.value());

    //     frSteerTarget = fr.angle.Radians().value();
    //     frDrive.Set(fr.speed.value());

    //     blSteerTarget = bl.angle.Radians().value();
    //     blDrive.Set(bl.speed.value());

    //     brSteerTarget = br.angle.Radians().value();
    //     brDrive.Set(br.speed.value());
    // }
    // else
    // {
    //     flDrive.Set(0);
    //     frDrive.Set(0);
    //     blDrive.Set(0);
    //     brDrive.Set(0);
    // }

    // Optimize swerve angles
    // frc::SwerveModuleState flO = frc::SwerveModuleState::Optimize(fl);
    // frc::SwerveModuleState frO = 
    // frc::SwerveModuleState blO = 
    // frc::SwerveModuleState brO = 




    // frc::SmartDashboard::PutNumber("FL Angle", fl.angle.Radians().value());
    // frc::SmartDashboard::PutNumber("FL Speed", fl.speed.value());
    
    // frc::SmartDashboard::PutNumber("FR Angle", fr.angle.Radians().value());
    // frc::SmartDashboard::PutNumber("FR Speed", fr.speed.value());

    // frc::SmartDashboard::PutNumber("BL Angle", bl.angle.Radians().value());
    // frc::SmartDashboard::PutNumber("BL Speed", bl.speed.value());

    // frc::SmartDashboard::PutNumber("BR Angle", br.angle.Radians().value());
    // frc::SmartDashboard::PutNumber("BR Speed", br.speed.value());

}

float moduloAngle(float angle)
{
    while (angle > M_PI) {angle -= M_PI * 2;}
    while (angle < -M_PI) {angle += M_PI * 2;}
    return angle;
}

void Drivetrain::printCalibationData()
{
    printf("Drivetrain Calibration Data\n");
    printf("DISABLE BEFORE COMPETITION!\n");
    printf("FL: %f\nFR: %f\nBL: %f\nBR: %f\n--\n",
        flEncoder.GetAbsolutePosition().GetValue().value(),
        frEncoder.GetAbsolutePosition().GetValue().value(),
        blEncoder.GetAbsolutePosition().GetValue().value(),
        brEncoder.GetAbsolutePosition().GetValue().value()
    );
}
