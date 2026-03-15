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
        frc::Pose2d getPose();
        void resetPose(frc::Pose2d pose);
        frc::ChassisSpeeds getRobotRelativeSpeeds();
        void driveRobotRelative(frc::ChassisSpeeds speed);
        frc::Rotation2d getRotation2d();
        frc::Pose2d startPose2d;
    private:

        subsystems::CommandSwerveDrivetrain driveTrain;
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
        frc::SwerveDrivePoseEstimator<4> swerveEstimator{kinematics,getRotation2d(),positions,startPose2d};
        double vx;
        double vy;
        double vOmega;
        swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.1).WithRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
        units::meters_per_second_t MaxSpeed = 1.0 * TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
        units::radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity
        units::meters_per_second_t get_max_speed(){return MaxSpeed;}
        units::radians_per_second_t get_max_angleRate(){return MaxAngularRate;}
        
};