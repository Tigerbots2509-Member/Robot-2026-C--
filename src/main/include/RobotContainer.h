// Copyright (c) FIRST and other WPILib contributors.
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
#include "subsystems/climber.h"
#include "frc/Joystick.h"
#include "frc2/command/button/JoystickButton.h"
#include "pathplanner/lib/auto/NamedCommands.h"
#include <frc2/command/RunCommand.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/RobotModeTriggers.h>
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
    double creepMult = 1;
    subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};
    frc2::CommandXboxController coPilot{3};
    frc2::CommandXboxController joystick{0};
    swerve::requests::RobotCentric aimeddrive = swerve::requests::RobotCentric{}
        .WithDeadband(MaxSpeed*0.1).WithRotationalDeadband(MaxAngularRate*0.1) //Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
        .WithSteerRequestType(swerve::SteerRequestType::Position);
    RobotContainer();
    units::meters_per_second_t get_max_speed(){return MaxSpeed;}
    units::radians_per_second_t get_max_angleRate(){return MaxAngularRate;}
    frc2::CommandPtr GetAutonomousCommand();
    void namedCommands();
private:
    void ConfigureBindings();
    double distance;
    intake Intake;
    launcher Launcher;
    hopperFeeder Hopper;
    climber Climber;
    
};
