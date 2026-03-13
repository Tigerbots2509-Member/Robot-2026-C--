//#include "Robot.h"
#include "LimelightHelpers.h"
#include <frc/DriverStation.h>
#include <math.h>
inline std::vector<LimelightHelpers::RawFiducial> getRawFiducials(std::string limelight_id)
{
    LimelightHelpers::PoseEstimate BotPoseEstimate = LimelightHelpers::getBotPoseEstimate(limelight_id, "botpose", false);
    return BotPoseEstimate.rawFiducials;
}
inline bool tagTargeting(int tagId,double* distance,double* angle){
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
inline bool rotationalValues(int tagId,double* distance,double* angle, double XCameraOffsetFromCenter, double YCameraOffsetFromCenter){
    double angleFromCamera;
    double horizontalFromCamera;
    double verticalFromCamera;
    double horizontalFromRobot;
    double verticalFromRobot;
    std::vector<LimelightHelpers::RawFiducial> aprilTagResults = getRawFiducials("limelight-b");
    for (LimelightHelpers::RawFiducial aprilTag : aprilTagResults){
        if(aprilTag.id == tagId){
            // *distance = aprilTag.distToCamera;
            angleFromCamera = aprilTag.txnc;
            horizontalFromCamera = aprilTag.tync;
            horizontalFromRobot = horizontalFromCamera + XCameraOffsetFromCenter;
            verticalFromCamera = std::sqrt(std::pow(aprilTag.distToCamera, 2) - std::pow(horizontalFromCamera, 2));
            verticalFromRobot = verticalFromCamera + YCameraOffsetFromCenter;

            *distance = std::sqrt(std::pow(verticalFromRobot, 2) + std::pow(horizontalFromRobot, 2));
            *angle = std::atan(horizontalFromRobot / verticalFromRobot);
            return true;
        }
    }

    //double cameraHypotonuse=sqrt(pow(distance*,2)/pow(ty,2));
    return false;
}
inline int ClosestHubId(std::string limelightId){
    int blueTags[4] = {20,24,26,18};
    int redTags[4] = {10,2,8,4};

    /*--- Filter AprilTag Ids By Alliance Color ---*/
    if(frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue){
        LimelightHelpers::SetFiducialIDFiltersOverride(limelightId,{24,26,18});
        
    }else if(frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed){
        LimelightHelpers::SetFiducialIDFiltersOverride(limelightId,{10,2,8});
    }
    /*--- Get Closest TagId ---*/
    int closetTag=LimelightHelpers::getFiducialID(limelightId);
    /*--- Reset April Tag Filtering ---*/
    LimelightHelpers::SetFiducialIDFiltersOverride(limelightId,{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32});
    /*--- Return closest AprilTag id, or 0 if none found ---*/
    return closetTag;
}