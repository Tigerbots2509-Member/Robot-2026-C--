// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
    ConfigureBindings();
    ConfigureAutoBuilder();
    GetAutonomousCommand();
    autoChooser = pathplanner::AutoBuilder::buildAutoChooserFilter([this] 
			(const pathplanner::PathPlannerAuto& autoCommand)
			{return autoCommand.GetName().starts_with("Comp");});
    //autoChooser.SetDefaultOption("CompDoNothing", pathplanner::AutoBuilder::buildAuto("CompDoNothing"));
	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}

void RobotContainer::ConfigureBindings(){

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed*creepMult) // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed*creepMult) // Drive left with negative X (left)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate*creepMult); // Drive counterclockwise with negative X (left)
        })
    );
    
    //(coPilot.LeftStick()||coPilot.RightStick()).WhileTrue(Launcher.wallOfBalls()).OnFalse(Launcher.launchZero());
    joystick.POVUp().WhileTrue(frc2::InstantCommand([this]{drivetrain.GetPigeon2().SetYaw(0.0_deg);}).ToPtr());
    joystick.LeftBumper().WhileTrue(frc2::InstantCommand([this]{creepMult=0.2;}).ToPtr()).OnFalse(frc2::InstantCommand([this]{creepMult=1;}).ToPtr());

    A7.WhileTrue(frc2::RunCommand([this]{Intake.intakeLiftDown();}).ToPtr().Until([this]{return Intake.liftMin.Get();}).AndThen([this]{frc2::InstantCommand([this]{Intake.intakeLiftStop();}).ToPtr();})).OnFalse(frc2::InstantCommand([this]{Intake.intakeLiftStop();}).ToPtr());
    A6.WhileTrue(frc2::RunCommand([this]{Intake.intakeLiftUp();}).ToPtr().Until([this]{return Intake.eLift.Get()<-700;}).AndThen([this]{frc2::InstantCommand([this]{Intake.intakeLiftStop();}).ToPtr();})).OnFalse(frc2::InstantCommand([this]{Intake.intakeLiftStop();}).ToPtr());
    
    A8.WhileTrue(frc2::RunCommand([this]{Intake.intakeIn();}).ToPtr()).OnFalse(frc2::InstantCommand([this]{Intake.intakeStop();}).ToPtr());
    //coPilot.RightBumper().WhileTrue(frc2::RunCommand([this]{Intake.intakeOut();}).ToPtr()).OnFalse(frc2::InstantCommand([this]{Intake.intakeStop();}).ToPtr());
    
    A1.WhileTrue(frc2::RunCommand([this]{Launcher.setLauncherSpeed(distance);}).ToPtr()).OnFalse(frc2::InstantCommand([this]{Launcher.launchZero();}).ToPtr());
    A2.WhileTrue(frc2::RunCommand([this]{Hopper.hopperToLauncher();}).ToPtr()).OnFalse(frc2::InstantCommand([this]{Hopper.hopperZero();}).ToPtr());
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    //joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    // joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
    //     return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    // }));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.

    joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

//FROM HERE DOWN IS AUTO STUFF
void RobotContainer::ApplyStart(){
    std::string autoName=autoChooser.GetSelected()->GetName();
    auto pathGroup = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(autoName);
    if(!pathGroup.empty()){
        pathplanner::AutoBuilder::resetOdom(pathGroup[0]->getStartingHolonomicPose().value());
        startPose=pathGroup[0]->getStartingHolonomicPose().value();
    }
}
void RobotContainer::namedCommands(){
    pathplanner::NamedCommands::registerCommand("Intake lift up", frc2::RunCommand([this]{Intake.intakeLiftUp();}).ToPtr().Until([this]{return Intake.eLift.Get()<-700;}).AndThen(frc2::InstantCommand([this]{Intake.intakeLiftStop();}).ToPtr()));
    pathplanner::NamedCommands::registerCommand("Intake lift down", frc2::RunCommand([this]{Intake.intakeLiftDown();}).ToPtr().Until([this]{return Intake.liftMin.Get();}).AndThen(frc2::InstantCommand([this]{Intake.intakeLiftStop();}).ToPtr()));

    pathplanner::NamedCommands::registerCommand("Intake in",frc2::RunCommand([this]{Intake.intakeIn();}).ToPtr().WithTimeout(3.5_s).AndThen(frc2::InstantCommand([this]{Intake.intakeStop();}).ToPtr()));
    
    pathplanner::NamedCommands::registerCommand("Launcher firing",frc2::RunCommand([this]{Hopper.hopperToLauncher();}).ToPtr().WithTimeout(3.5_s).AndThen(frc2::RunCommand([this]{Launcher.setLauncherSpeed(distance);}).ToPtr()).WithTimeout(5.5_s).AndThen(frc2::InstantCommand([this]{Launcher.launchZero();}).ToPtr()).AndThen(frc2::InstantCommand([this]{Hopper.hopperZero();}).ToPtr()));
}
void RobotContainer::ConfigureAutoBuilder(){
    // Do all subsystem initialization here
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();
    // Configure the AutoBuilder last
    pathplanner::AutoBuilder::configure(
        [this](){ return drivetrain.GetState().Pose; }, // Robot pose supplier
        [this](frc::Pose2d pose){ drivetrain.ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return drivetrain.GetState().Speeds; }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds, pathplanner::DriveFeedforwards feedforwards){ drivetrain.SetControl(autoRobotSpeedsRequest.WithSpeeds(frc::ChassisSpeeds::Discretize(speeds.vx,speeds.vy,speeds.omega, 0.020_s))
            .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
            .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY));}, 
        
        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<pathplanner::PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            pathplanner::PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
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
        &drivetrain // Reference to this subsystem to set requirements
    );
    poseEstimator= new frc::SwerveDrivePoseEstimator<4>{kinematics,frc::Rotation2d{},{drivetrain.GetModule(0).GetPosition(false),drivetrain.GetModule(1).GetPosition(false),drivetrain.GetModule(2).GetPosition(false),drivetrain.GetModule(3).GetPosition(false)},frc::Pose2d{},{0,0,0},{0,0,0}};
    poseEstimator->ResetPose(startPose);
}
frc2::Command* RobotContainer::GetAutonomousCommand(){
    //This is the pathplanner "load an auto"
    return autoChooser.GetSelected();
}
void RobotContainer::Periodic(){
    // if (frc::DriverStation::IsDisabled()){
    //     poseEstimator->ResetPose(startPose);
    //     drivetrain.GetPigeon2().SetYaw(poseEstimator->GetEstimatedPosition().Rotation().Degrees());
    // }
    
}