#include "subsystems/Auto.h"


AutoManager automan;

photon::PhotonPoseEstimator estimator(automan.aprilTagFieldLayout, photon::CLOSEST_TO_REFERENCE_POSE, automan.robotToCam);

// std::pair<frc::Pose2d, units::millisecond_t> getEstimatedGlobalPose(
//     frc::Pose3d prevEstimatedRobotPose, photon::PhotonPipelineResult neededresult) {
//   estimator.SetReferencePose(prevEstimatedRobotPose);
//   units::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
//   std::optional result = estimator.Update(neededresult);
//   if (result.second) {
//     return std::make_pair<>(result.first.ToPose2d(),
//                             currentTime - result.second);
//   } else {
//     return std::make_pair(frc::Pose2d(), 0_ms);
//   }
// }


// TODO