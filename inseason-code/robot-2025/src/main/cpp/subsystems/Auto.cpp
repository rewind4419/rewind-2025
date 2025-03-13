#include "subsystems/Auto.h"


AutoManager automan;


photon::PhotonPoseEstimator estimator(automan.aprilTagFieldLayout, photon::CLOSEST_TO_REFERENCE_POSE, automan.robotToCam);

/*
* Returns an std::optional of the estimated robot pose.
* Requires a photon pipeline result.
*/
std::optional<photon::EstimatedRobotPose> AutoManager::Estimate(photon::PhotonPipelineResult result){
    return estimator.Update(result);
}