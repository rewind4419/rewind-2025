#include "Vision.h"
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>


//frc::Field2d m_field {};

frc::Pose2d REL_RIGHT_SCORE { 0.41760_m, -0.7738_m, frc::Rotation2d {0_rad} };

void VisionManager::Init()
{

  //frc::SmartDashboard::PutData("Field", &m_field);
}

/*
* Returns an std::optional of the estimated robot pose.
* Requires a photon pipeline result.
*/
std::optional<photon::EstimatedRobotPose> VisionManager::Estimate(photon::PhotonPipelineResult result){
  return this->estimator.Update(result);
}

frc::Pose2d VisionManager::TagToWorld(frc::Pose2d offsetFromTag, int tagId)
{
  std::optional<frc::Pose3d> tagPoseOptional = aprilTagFieldLayout.GetTagPose(tagId);
  if(!tagPoseOptional.has_value())
  {
    printf("Uh oh: Tag with ID: %i doesn't exist or something (optional ! has value)\n",tagId);
    return frc::Pose2d();
  }
  frc::Pose3d tagPose = tagPoseOptional.value();

  double tagAngle = tagPose.Rotation().Z().value();

  double dirX = cos(tagAngle);
  double dirY = sin(tagAngle);
  
  ///tagPose + dir * X + perpDir * Y;

  frc::Pose2d outputPose {
    tagPose.X() + dirX * offsetFromTag.X() - dirY * offsetFromTag.Y(),
    tagPose.Y() + dirY * offsetFromTag.X() + dirX * offsetFromTag.Y(),
    tagAngle * 1_rad + offsetFromTag.Rotation().Radians() + M_PI*1_rad
  };
  return outputPose;
}

frc::Pose2d VisionManager::WorldToTag(frc::Pose2d worldPosition, int tagId)
{
  
  std::optional<frc::Pose3d> tagPoseOptional = aprilTagFieldLayout.GetTagPose(tagId);
  if(!tagPoseOptional.has_value())
  {
    printf("Uh oh: Tag with ID: %i doesn't exist or something (optional ! has value)\n",tagId);
    return frc::Pose2d();
  }
  frc::Pose3d tagPose = tagPoseOptional.value();

  double tagAngle = tagPose.Rotation().Z().value();

  double dirX = cos(tagAngle);
  double dirY = sin(tagAngle);

  double worldX = worldPosition.X().value() - tagPose.X().value();
  double worldY = worldPosition.Y().value() - tagPose.Y().value();

  // unroteMat * world =  tagOffset
  ///tagPose + (dir * X + perpDir * Y);

  frc::Pose2d outputPose {
     (dirX * worldX + dirY * worldY) * 1.0_m,
    (-dirY * worldX + dirX * worldY) * 1.0_m,
     worldPosition.Rotation() - M_PI*1_rad - tagAngle * 1_rad 
  };
  return outputPose;
}

void VisionManager::updRoutine(){
  photon::PhotonPipelineResult result = this->Cam1.GetLatestResult();
  photon::PhotonTrackedTarget target = result.GetBestTarget();

  photon::PhotonPipelineResult filteredResults = result;

  filteredResults.targets.clear();

  // printf("huhuh? timestamp: %i\n",filteredResults.GetTimestamp());

  frc::SmartDashboard::PutNumber("Tag Set", this->tagsAllowed);
  
  for(int i = 0; i < result.GetTargets().size(); i++)
  {
    int id = result.GetTargets()[i].fiducialId ;
    if(
      (id >= 6 && id <= 11 && tagsAllowed == RED_REEF) ||
      (id >= 17 && id <= 22 && tagsAllowed == BLUE_REEF) ||
      tagsAllowed == ALL
    )
    {
      filteredResults.targets.push_back(result.GetTargets()[i]);
    }
  }
  bool hasTargets = filteredResults.HasTargets();
  this->optionalVisionEstimate = this->Estimate(filteredResults);



  frc::SmartDashboard::PutNumber("Tags Detected (vision disabled in teleop)", result.targets.size());

  if (hasTargets){
    //printf("Got a target, checking if it has a value\n");
    if (this->optionalVisionEstimate.has_value()){
      //printf("I got a value!, %f, %f\n", this->optional.value().estimatedPose.ToPose2d().X(), this->optional.value().estimatedPose.ToPose2d().Y());
      
      //m_field.SetRobotPose(this->optional.value().estimatedPose.ToPose2d());
    // photon::EstimatedRobotPose estimation = visionManager.Estimate(result).value();
        frc::SmartDashboard::PutNumber("Photon pose X", this->optionalVisionEstimate.value().estimatedPose.X().value());
        frc::SmartDashboard::PutNumber("Photon pose Y", this->optionalVisionEstimate.value().estimatedPose.Y().value());
        frc::SmartDashboard::PutNumber("Photon pose R", this->optionalVisionEstimate.value().estimatedPose.Rotation().Z().value());

        frc::Pose2d robotPose = this->optionalVisionEstimate.value().estimatedPose.ToPose2d();

        frc::Pose2d tag7Pose = WorldToTag(robotPose, 6);

        frc::Pose2d tag7BackToWorld = TagToWorld(tag7Pose, 6);

        frc::SmartDashboard::PutNumber("Tag 11 Relative X", tag7Pose.X().value());
        frc::SmartDashboard::PutNumber("Tag 11 Relative Y", tag7Pose.Y().value());
        frc::SmartDashboard::PutNumber("Tag 11 Relative Rot", tag7Pose.Rotation().Radians().value());

        frc::SmartDashboard::PutNumber("Tag 7 Back to world X", tag7BackToWorld.X().value());
        frc::SmartDashboard::PutNumber("Tag 7 Back to world Y", tag7BackToWorld.Y().value());
        frc::SmartDashboard::PutNumber("Tag 7 Back to world Rot", tag7BackToWorld.Rotation().Radians().value());
    } else {
      // printf("No value here.   ");
    }
  }
}