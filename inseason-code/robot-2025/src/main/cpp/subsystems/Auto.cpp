#include "subsystems/Auto.h"
#include <frc/smartdashboard/Field2d.h>
#include <frc/SmartDashboard/SmartDashboard.h>

frc::Field2d m_field;




void AutoManager::Init()
{
  frc::SmartDashboard::PutData("Field", &m_field);
}

/*
* Returns an std::optional of the estimated robot pose.
* Requires a photon pipeline result.
*/
std::optional<photon::EstimatedRobotPose> AutoManager::Estimate(photon::PhotonPipelineResult result){
    return this->estimator.Update(result);
}
void AutoManager::updRoutine(){
  photon::PhotonPipelineResult result = this->Cam1.GetLatestResult();
  bool hasTargets = result.HasTargets();
  photon::PhotonTrackedTarget target = result.GetBestTarget();
  this->optional = this->Estimate(result);

  if (hasTargets){
    //printf("Got a target, checking if it has a value\n");
    if (this->optional.has_value()){
      printf("I got a value!, %f, %f\n", this->optional.value().estimatedPose.ToPose2d().X(), this->optional.value().estimatedPose.ToPose2d().Y());
      
      m_field.SetRobotPose(this->optional.value().estimatedPose.ToPose2d());
    // photon::EstimatedRobotPose estimation = m.Estimate(result).value();
        frc::SmartDashboard::PutNumber("Photon pose X", this->optional.value().estimatedPose.X().value());
        frc::SmartDashboard::PutNumber("Photon pose Y", this->optional.value().estimatedPose.Y().value());
        frc::SmartDashboard::PutNumber("Photon pose Z", this->optional.value().estimatedPose.Z().value());
    } else {
      // printf("No value here.   ");
    }
  }
}