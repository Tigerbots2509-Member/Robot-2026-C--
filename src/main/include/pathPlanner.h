#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <subsystems/CommandSwerveDrivetrain.h>
#include <frc/geometry/Pose2d.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <RobotContainer.h>

class pathPlanner{
    public:
        pathPlanner();
        frc::Pose2d startPose2d{0_m,0_m,frc::Rotation2d{0_deg}};
    private:
        subsystems::CommandSwerveDrivetrain driveTrain{TunerConstants::CreateDrivetrain()};
        RobotContainer container;
        std::array<frc::SwerveModuleState,4> states{
            driveTrain.GetModule(0).GetCurrentState(),
            driveTrain.GetModule(1).GetCurrentState(),
            driveTrain.GetModule(2).GetCurrentState(),
            driveTrain.GetModule(3).GetCurrentState()
        };
        std::array<frc::SwerveModulePosition,4> positions{
            driveTrain.GetModule(0).GetPosition(false),
            driveTrain.GetModule(1).GetPosition(false),
            driveTrain.GetModule(2).GetPosition(false),
            driveTrain.GetModule(3).GetPosition(false)
        };
        frc::Rotation2d rotation2d = driveTrain.GetPigeon2().GetRotation2d();
        // Locations for the swerve drive modules relative to the robot center.
        frc::Translation2d m_frontLeftLocation{8.5_in, 8.5_in};
        frc::Translation2d m_frontRightLocation{8.5_in, -8.5_in};
        frc::Translation2d m_backLeftLocation{-8.5_in, 8.5_in};
        frc::Translation2d m_backRightLocation{-8.5_in, -8.5_in};
        // Creating my kinematics object using the module locations.
        frc::SwerveDriveKinematics<4> kinematics{
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
        m_backRightLocation};
        swerve::requests::ApplyRobotSpeeds autoRobotSpeedsRequest = swerve::requests::ApplyRobotSpeeds();
        frc::SwerveDrivePoseEstimator<4> poseEstimator{kinematics,frc::Rotation2d{},positions,frc::Pose2d{},{0.0,0.0,0.0},{0.0,0.0,0.0}};
};