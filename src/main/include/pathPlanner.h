#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <subsystems/CommandSwerveDrivetrain.h>
#include <frc/geometry/Pose2d.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <RobotContainer.h>

#include <wpi/array.h>

class pathPlanner{
    public:
        pathPlanner();
        //frc::Rotation2d getRotation2d();
        //frc::Pose2d startPose2d;
    private:
        subsystems::CommandSwerveDrivetrain driveTrain{TunerConstants::CreateDrivetrain()};
        RobotContainer container;
        std::array<frc::SwerveModuleState,4> states{
            driveTrain.GetModule(0).GetCurrentState(),
            driveTrain.GetModule(1).GetCurrentState(),
            driveTrain.GetModule(2).GetCurrentState(),
            driveTrain.GetModule(3).GetCurrentState()};
        
        std::array<frc::SwerveModulePosition,4> positions{
            driveTrain.GetModule(0).GetPosition(false),
            driveTrain.GetModule(1).GetPosition(false),
            driveTrain.GetModule(2).GetPosition(false),
            driveTrain.GetModule(3).GetPosition(false)};
        frc::SwerveDriveKinematics<4> kinematics{
            frc::Translation2d(0.3048_m,0.3048_m),
            frc::Translation2d(0.3048_m,-0.3048_m),
            frc::Translation2d(-0.3048_m,0.3048_m),
            frc::Translation2d(-0.3048_m,-0.3048_m),
        };
        swerve::requests::ApplyRobotSpeeds autoRobotSpeedsRequest = swerve::requests::ApplyRobotSpeeds();
        //frc::SwerveDrivePoseEstimator<4> swerveEstimator{kinematics,getRotation2d(),positions,startPose2d};        
};