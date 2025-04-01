#pragma once
#include <photon/PhotonCamera.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonUtils.h>
#include <photon/estimation/VisionEstimation.h>

extern frc::Pose2d REL_RIGHT_SCORE;

enum TagSet
{
  RED_REEF,
  BLUE_REEF,
  ALL,
};

class VisionManager
{
public:
    void Init();

    photon::PhotonCamera Cam1{"Cam1"};
    frc::Transform3d robotToCam = frc::Transform3d(frc::Translation3d(0.2579_m, -0.2504_m, 0.2126_m), frc::Rotation3d(-0.087_rad, -0.349_rad, 0.524_rad));//frc::Rotation3d(0_rad, 0.436_rad, 0.486_rad));
    frc::AprilTagFieldLayout aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2025Reefscape);
    std::optional<photon::EstimatedRobotPose> Estimate(photon::PhotonPipelineResult result);

    frc::Pose2d TagToWorld(frc::Pose2d offsetFromTag, int tagId);
    frc::Pose2d WorldToTag(frc::Pose2d worldPosition, int tagId);

    void updRoutine();
    std::optional<photon::EstimatedRobotPose> optionalVisionEstimate;
    
    TagSet tagsAllowed = ALL;

    
    photon::PhotonPoseEstimator estimator{this->aprilTagFieldLayout, photon::MULTI_TAG_PNP_ON_COPROCESSOR, this->robotToCam};
    //photon::PhotonPoseEstimator estimator;
};
