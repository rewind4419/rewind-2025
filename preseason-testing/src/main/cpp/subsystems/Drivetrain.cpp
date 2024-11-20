#include "subsystems/Drivetrain.h"

#include <stdio.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/smartdashboard/SmartDashboard.h>

Drivetrain::Drivetrain()
{
    
}

void Drivetrain::SetVec(float x, float y, float r)
{
    frc::ChassisSpeeds speeds {
        units::meters_per_second_t{x}, 
        units::meters_per_second_t{y}, 
        units::radians_per_second_t{r}
    };

    wpi::array<frc::SwerveModuleState, 4U> states = kinematics.ToSwerveModuleStates(speeds);

    frc::SwerveModuleState fl = states.at(0);
    frc::SwerveModuleState fr = states.at(1);
    frc::SwerveModuleState bl = states.at(2);
    frc::SwerveModuleState br = states.at(3);

    // Optimize swerve angles
    // frc::SwerveModuleState flO = frc::SwerveModuleState::Optimize(fl);
    // frc::SwerveModuleState frO = 
    // frc::SwerveModuleState blO = 
    // frc::SwerveModuleState brO = 

    // frc::SmartDashboard::PutNumber("FL Angle", fl.angle.Degrees().value());
    // frc::SmartDashboard::PutNumber("FL Speed", fl.speed.value());
    
    // frc::SmartDashboard::PutNumber("FR Angle", fr.angle.Degrees().value());
    // frc::SmartDashboard::PutNumber("FR Speed", fr.speed.value());

    // frc::SmartDashboard::PutNumber("BL Angle", bl.angle.Degrees().value());
    // frc::SmartDashboard::PutNumber("BL Speed", bl.speed.value());

    // frc::SmartDashboard::PutNumber("BR Angle", br.angle.Degrees().value());
    // frc::SmartDashboard::PutNumber("BR Speed", br.speed.value());

    frc::SmartDashboard::PutNumber("FL Angle", flEncoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("FR Angle", frEncoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("BL Angle", blEncoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("BR Angle", brEncoder.GetAbsolutePosition());

    frDrive.Set(frc::SmartDashboard::GetNumber("frDrive", 0.0));
    flDrive.Set(frc::SmartDashboard::GetNumber("flDrive", 0.0));
    brDrive.Set(frc::SmartDashboard::GetNumber("brDrive", 0.0));
    blDrive.Set(frc::SmartDashboard::GetNumber("blDrive", 0.0));

    frSteer.Set(frc::SmartDashboard::GetNumber("frSteer", 0.0));
    flSteer.Set(frc::SmartDashboard::GetNumber("flSteer", 0.0));
    brSteer.Set(frc::SmartDashboard::GetNumber("brSteer", 0.0));
    blSteer.Set(frc::SmartDashboard::GetNumber("blSteer", 0.0));
}

