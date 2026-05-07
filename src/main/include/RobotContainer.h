// Copyright (c) FIRST and other WLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Telemetry.h"
#include "subsystems/intake.h"
#include "subsystems/launcher.h"
#include "subsystems/hopperFeeder.h"
#include "frc/Joystick.h"
#include "frc2/command/button/JoystickButton.h"
#include "pathplanner/lib/auto/NamedCommands.h"
#include "frc2/command/ParallelCommandGroup.h"
#include <frc2/command/RunCommand.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc/smartdashboard/Field2d.h>
//Check for double including
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <subsystems/CommandSwerveDrivetrain.h>
#include <frc/geometry/Pose2d.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc2/command/Command.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <memory>

class RobotContainer {
private:
    units::meters_per_second_t MaxSpeed = 1.0 * TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.1).WithRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors

    // swerve::requests::RobotCentric aimedDrive = swerve::requests::RobotCentric{}
    //     .WithDeadband(MaxSpeed*0.1).WithRotationalDeadband(MaxAngularRate*0.1) //Add a 10% deadband
    //     .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
    //     .WithSteerRequestType(swerve::SteerRequestType::Position);

    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{MaxSpeed};
    

public:
    std::array<frc::SwerveModulePosition,4> positions{
        drivetrain.GetModule(0).GetPosition(false),
        drivetrain.GetModule(1).GetPosition(false),
        drivetrain.GetModule(2).GetPosition(false),
        drivetrain.GetModule(3).GetPosition(false)
    };
    void Periodic();
    frc::Field2d field;
    frc::Pose2d visionPose2d;
    frc::SendableChooser<frc2::Command*> autoChooser;
    frc::SwerveDrivePoseEstimator<4>* poseEstimator; 
    double creepMult = 1;
    subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};
    frc::Joystick boardA{1};
    frc::Joystick boardB{2};
    frc2::CommandXboxController joystick{0};
    swerve::requests::RobotCentric aimeddrive = swerve::requests::RobotCentric{}
        .WithDeadband(MaxSpeed*0.1).WithRotationalDeadband(MaxAngularRate*0.1) //Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
        .WithSteerRequestType(swerve::SteerRequestType::Position);
    RobotContainer();
    units::meters_per_second_t get_max_speed(){return MaxSpeed;}
    units::radians_per_second_t get_max_angleRate(){return MaxAngularRate;}
    frc2::Command* GetAutonomousCommand();
    void namedCommands();
    frc2::JoystickButton A1{&boardA,1};
    frc2::JoystickButton A2{&boardA,2};
    frc2::JoystickButton A3{&boardA,3};
    frc2::JoystickButton A4{&boardA,4};
    frc2::JoystickButton A5{&boardA,5};
    frc2::JoystickButton A6{&boardA,6};
    frc2::JoystickButton A7{&boardA,7};
    frc2::JoystickButton A8{&boardA,8};
    frc2::JoystickButton B1{&boardB,1};
    frc2::JoystickButton B2{&boardB,2};
    frc2::JoystickButton B3{&boardB,3};
    frc2::JoystickButton B4{&boardB,4};
    frc2::JoystickButton B5{&boardB,5};
    frc2::JoystickButton B6{&boardB,6};
    frc2::JoystickButton B7{&boardB,7};
private:
    void ConfigureBindings();
    void ConfigureAutoBuilder();
    void ApplyStart();
    frc::SwerveModuleState getStates();
    frc::SendableChooser<frc2::Command*> GetSelection();
    double distance;
    intake Intake;
    launcher Launcher;
    hopperFeeder Hopper;
    //From here down is the auto stuff
    //frc::SendableChooser<frc2::Command> autoChooser;
    
    
    frc::Pose2d startPose=frc::Pose2d{0_m,0_m,0_deg};
    frc::Rotation2d rotation2d = drivetrain.GetPigeon2().GetRotation2d();
        // Locations for the swerve drive modules relative to the robot center.
    frc::Translation2d m_frontLeftLocation{8.5_in, 8.5_in};
    frc::Translation2d m_frontRightLocation{8.5_in, -8.5_in};
    frc::Translation2d m_backLeftLocation{-8.5_in, 8.5_in};
    frc::Translation2d m_backRightLocation{-8.5_in, -8.5_in};
        // Creating my kinematics object using the module locations.
    frc::SwerveDriveKinematics<4> kinematics{
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,m_backRightLocation};
    swerve::requests::ApplyRobotSpeeds autoRobotSpeedsRequest = swerve::requests::ApplyRobotSpeeds();
    
};
