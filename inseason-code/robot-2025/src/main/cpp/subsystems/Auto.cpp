#include "subsystems/Auto.h"
#include <frc/smartdashboard/Field2d.h>
#include <frc/SmartDashboard/SmartDashboard.h>

frc::Field2d m_field;
AutoManager automan;



photon::PhotonPoseEstimator estimator(automan.aprilTagFieldLayout, photon::CLOSEST_TO_REFERENCE_POSE, automan.robotToCam);

/*
* Returns an std::optional of the estimated robot pose.
* Requires a photon pipeline result.
*/
std::optional<photon::EstimatedRobotPose> AutoManager::Estimate(photon::PhotonPipelineResult result){
    return estimator.Update(result);
}
void updRoutine(){
photon::PhotonPipelineResult result = automan.Cam1.GetLatestResult();
  bool hasTargets = result.HasTargets();
  photon::PhotonTrackedTarget target = result.GetBestTarget();
  automan.optional = automan.Estimate(result);
  if (hasTargets){
    // printf("Got a target, checking if it has a value\n");
    if (automan.optional.has_value()){
      // printf("I got a value!\n");
      m_field.SetRobotPose(automan.optional.value().estimatedPose.ToPose2d());
    // photon::EstimatedRobotPose estimation = m.Estimate(result).value();
        frc::SmartDashboard::PutNumber("Photon pose X", automan.optional.value().estimatedPose.X().value());
        frc::SmartDashboard::PutNumber("Photon pose Y", automan.optional.value().estimatedPose.Y().value());
        frc::SmartDashboard::PutNumber("Photon pose Z", automan.optional.value().estimatedPose.Z().value());
    } else {
      // printf("No value here.   ");
    }
  }
}