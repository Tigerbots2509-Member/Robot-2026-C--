#include "Robot.h" 

std::vector<LimelightHelpers::RawFiducial> getRawFiducials(std::string limelight_id)
{
    LimelightHelpers::PoseEstimate BotPoseEstimate = LimelightHelpers::getBotPoseEstimate(limelight_id, "botpose", false);
    return BotPoseEstimate.rawFiducials;
}
bool tagTargeting(int tagId,double* distance,double* angle){
    std::vector<LimelightHelpers::RawFiducial> aprilTagResults = getRawFiducials("limelight-b");
    for (LimelightHelpers::RawFiducial aprilTag : aprilTagResults){
        if(aprilTag.id==tagId){
            *distance=aprilTag.distToCamera;
            *angle=aprilTag.txnc;
            return true;
        }
    }
    return false;
}