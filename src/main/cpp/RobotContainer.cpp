// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"


RobotContainer::RobotContainer()
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{

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

    coPilot.POVDown().WhileTrue(frc2::RunCommand([this]{Climber.climbDown();}).ToPtr().Until([this]{return Climber.minClimb.Get();}).AndThen([this]{frc2::InstantCommand([this]{Climber.climbZero();});})).OnFalse(frc2::InstantCommand([this]{Climber.climbZero();}).ToPtr());
    coPilot.POVUp().WhileTrue(frc2::RunCommand([this]{Climber.climbUp();}).ToPtr()).OnFalse(frc2::InstantCommand([this]{Climber.climbZero();}).ToPtr());

    coPilot.A().WhileTrue(frc2::RunCommand([this]{Intake.intakeLiftDown();}).ToPtr().Until([this]{return Intake.liftMax.Get();}).AndThen([this]{frc2::InstantCommand([this]{Intake.intakeLiftStop();}).ToPtr();})).OnFalse(frc2::InstantCommand([this]{Intake.intakeLiftStop();}).ToPtr());
    coPilot.Y().WhileTrue(frc2::RunCommand([this]{Intake.intakeLiftUp();}).ToPtr().Until([this]{return Intake.eLift.Get()<-700;}).AndThen([this]{frc2::InstantCommand([this]{Intake.intakeLiftStop();}).ToPtr();})).OnFalse(frc2::InstantCommand([this]{Intake.intakeLiftStop();}).ToPtr());
    
    coPilot.LeftBumper().WhileTrue(frc2::RunCommand([this]{Intake.intakeIn();}).ToPtr()).OnFalse(frc2::InstantCommand([this]{Intake.intakeStop();}).ToPtr());
    coPilot.RightBumper().WhileTrue(frc2::RunCommand([this]{Intake.intakeOut();}).ToPtr()).OnFalse(frc2::InstantCommand([this]{Intake.intakeStop();}).ToPtr());
    
    coPilot.RightTrigger().WhileTrue(frc2::RunCommand([this]{Launcher.setLauncherSpeed(distance);}).ToPtr()).OnFalse(frc2::InstantCommand([this]{Launcher.launchZero();}).ToPtr());
    coPilot.LeftTrigger().WhileTrue(frc2::RunCommand([this]{Hopper.hopperToLauncher();}).ToPtr()).OnFalse(frc2::InstantCommand([this]{Hopper.hopperZero();}).ToPtr());
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
    (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    // Simple drive forward auton
    return frc2::cmd::Sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(frc::Rotation2d{0_deg}); }),
        // Then slowly drive forward (away from us) for 5 seconds.
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(0.5_mps)
                .WithVelocityY(0_mps)
                .WithRotationalRate(0_tps);
        })
        .WithTimeout(5_s),
        // Finally idle for the rest of auton
        drivetrain.ApplyRequest([] { return swerve::requests::Idle{}; })
    );
}
