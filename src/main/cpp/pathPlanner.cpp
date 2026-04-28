#include <pathPlanner.h>
using namespace pathplanner;
pathPlanner::pathPlanner(){
    // Do all subsystem initialization here

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = RobotConfig::fromGUISettings();
    // Configure the AutoBuilder last
    AutoBuilder::configure(
        [this](){ return driveTrain.GetState().Pose; }, // Robot pose supplier
        [this](frc::Pose2d pose){ driveTrain.ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return driveTrain.GetState().Speeds; }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds, pathplanner::DriveFeedforwards feedforwards){ driveTrain.SetControl(autoRobotSpeedsRequest.WithSpeeds(frc::ChassisSpeeds::Discretize(speeds.vx,speeds.vy,speeds.omega, 0.020_s))
            .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
            .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY));}, 
        
        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        &driveTrain // Reference to this subsystem to set requirements
    );
}