#pragma once
#include <photon/PhotonCamera.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonUtils.h>
#include <photon/estimation/VisionEstimation.h>

class AutoManager
{
public:
    photon::PhotonCamera Cam1{"Cam1"};
    frc::Transform3d robotToCam = frc::Transform3d(frc::Translation3d(0.30_m, 0.28_m, 0.17_m), frc::Rotation3d(0_rad, 0_rad, 0_rad));
    frc::AprilTagFieldLayout aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2025Reefscape);
    std::optional<photon::EstimatedRobotPose> Estimate(photon::PhotonPipelineResult result);
};

 