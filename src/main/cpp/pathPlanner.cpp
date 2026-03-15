#include <pathPlanner.h>
using namespace pathplanner;
pathPlanner::pathPlanner(){
    // Do all subsystem initialization here
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = RobotConfig::fromGUISettings();

    // Configure the AutoBuilder last
    AutoBuilder::configure(
        [this](){ return getPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ resetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards){ driveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
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
        this // Reference to this subsystem to set requirements
    );
}
frc::Pose2d pathPlanner::getPose(){
    frc::Pose2d pose=swerveEstimator.GetEstimatedPosition();
    return pose;
}
void pathPlanner::resetPose(frc::Pose2d pose){
    driveTrain.ResetPose(pose);
}
frc::ChassisSpeeds pathPlanner::getRobotRelativeSpeeds(){
    return kinematics.ToChassisSpeeds(states);
}
void pathPlanner::driveRobotRelative(frc::ChassisSpeeds speed){
    vx=speed.vx();
    vy=speed.vy();
    vOmega=speed.omega();
    if(frc::DriverStation::IsAutonomous()){
        container.drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(vx* MaxSpeed*0.5) // Drive forward with negative Y (forward)
                .WithVelocityY(vy * MaxSpeed*0.5) // Drive left with negative X (left)
                .WithRotationalRate(vOmega* MaxAngularRate*0.5); // Drive counterclockwise with negative X (left)
        });   
    }
}
frc::Rotation2d pathPlanner::getRotation2d(){
    frc::Rotation2d pose2=driveTrain.GetPigeon2().GetRotation2d();
    return pose2;
}