#include "subsystems/SwerveDrivetrain.h"
#include <frc/RobotController.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/SwerveDrivetrain.h"

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */

   
    // if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
    //     auto const allianceColor = frc::DriverStation::GetAlliance();
    //     if (allianceColor) {
    //         SetOperatorPerspectiveForward(
    //             *allianceColor == frc::DriverStation::Alliance::kRed
    //                 ? kRedAlliancePerspectiveRotation
    //                 : kBlueAlliancePerspectiveRotation
    //         );
    //         m_hasAppliedOperatorPerspective = true;
    //     }
    // }

    SetOperatorPerspectiveForward(
                kRedAlliancePerspectiveRotation);

    frc::Pose2d pose = this->GetState().Pose;
    
    frc::SmartDashboard::PutNumber("Pose X", pose.X().value());
    frc::SmartDashboard::PutNumber("Pose Y", pose.Y().value());
    frc::SmartDashboard::PutNumber("Pose R", this->GetState().Pose.Rotation().Radians().value());

    frc::SmartDashboard::PutNumber("Vel X", this->GetState().Speeds.vx.value());
    frc::SmartDashboard::PutNumber("Vel Y", this->GetState().Speeds.vy.value());
    frc::SmartDashboard::PutNumber("Vel R", this->GetState().Speeds.omega.value());

    //frc::SmartDashboard::PutNumber("IMU Angle", this->GetPigeon2().GetAngle());
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}
