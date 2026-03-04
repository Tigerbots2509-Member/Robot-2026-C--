// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc2/command/CommandScheduler.h>

Robot::Robot(){}

void Robot::RobotPeriodic() {
    m_timeAndJoystickReplay.Update();
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand.value());
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand.value());
    }
}

void Robot::TeleopPeriodic() {
    double strafe;
    double lateral;
    double distance;//value to tell you how far your camera is from the april tag
    double AprilTagAngle;
    double rotation;//rotational input for drivetrain
    double angleDerivative= 0.05;//These are tuners       it works don't question
    double angleIntegral=0;//frc::SmartDashboard::GetNumber("Integral", 0);//These are tuners          it smooths the motion on the robot.
    double angleProptional=0.05;//frc::SmartDashboard::GetNumber("Proptional", 0);//These are tuners 
    double distProptional=0.75; //Range of values to adjust to so increase makes output range higher and vice versa
    double distIntegral=0; //Doesn't get used very much in this situation, smooths out curve
    double distDerivative=0; //Rate of change of your proportional, smooths out the change
    frc::PIDController distTargetingPID(distProptional,distIntegral,distDerivative); //PID to maintain dist to target
    frc::PIDController angleTargetingPID{angleProptional,angleIntegral,angleDerivative}; //PID to maintain angle to target
    if(driverController.GetAButton()){
        frc::SmartDashboard::PutBoolean("TV", driverController.GetAButton());
        if(tagTargeting(12, &distance, &rotation)){
            frc::SmartDashboard::PutNumber("Angle PID Result:", .1*angleTargetingPID.Calculate(rotation,0));
            rotation = .1*angleTargetingPID.Calculate(rotation,0);
            frc::SmartDashboard::PutNumber("Distance PID Result:", distTargetingPID.Calculate(distance,1));
            lateral = distTargetingPID.Calculate(distance,1);

            m_container.drivetrain.SetControl(
              m_container.aimedDrive.WithVelocityX(lateral* m_container.get_max_speed())
                                    .WithVelocityY(strafe*m_container.get_max_speed())
                                    .WithRotationalRate(rotation*m_container.get_max_angleRate())
            );

            //DriveTrain.tankDrive(-1*distTargetingPID.Calculate(distance,1.5),-1*angleTargetingPID.Calculate(rotation,0)); // While the A button is pressed it gets your dist and angle from ID 12 to maintain 1.5m dist and 0 degree angle
            frc::SmartDashboard::PutNumber("Rotation:",rotation);
            frc::SmartDashboard::PutNumber("Distance:",distance);
            frc::SmartDashboard::PutNumber("power", angleTargetingPID.Calculate(rotation,0));
        }else{
            //DriveTrain.tankDrive(0, 0);
    }
    }else{
        lateral = -driverController.GetLeftY();
        rotation = driverController.GetRightX();
        frc::SmartDashboard::PutBoolean("TV", driverController.GetAButton());
        //DriveTrain.tankDrive(0, angleTargetingPID.Calculate(0,0));
  }
    if (driverController.GetXButton()) {        //Wouldn't we want this apart of the drivers a button and not a seperate
        rotationalValues(26, &distance, &AprilTagAngle, 5.75, 10);
        frc::SmartDashboard::PutNumber("Distance To AprilTag", distance);
        frc::SmartDashboard::PutNumber("Angle to AprilTag", AprilTagAngle);
    };
    if(coPilot.GetLeftTriggerAxis()>0.25){
        Hopper.hopperToLauncher();
    }else{
        Hopper.hopperZero();
    }

    if(coPilot.GetLeftStickButton()||coPilot.GetRightStickButton()){
        Launcher.wallOfBalls();
    }else if(coPilot.GetRightTriggerAxis()>=0.25){
        Launcher.launchByPower();
    }else{
        Launcher.launchZero();
    }

    if(coPilot.GetAButton()){
        Intake.intakeLiftDown();
    }else if(coPilot.GetYButton()){
        Intake.intakeLiftUp();
    }else{
        Intake.intakeLiftStop();
    }
    
    if(coPilot.GetLeftBumper()){
        Intake.intakeIn();
    }else if(coPilot.GetRightBumper()){
        Intake.intakeOut();
    }else{
        Intake.intakeStop();
    };

    if(coPilot.GetPOV()==0){
        Climber.climbUp();
    }else if(coPilot.GetPOV()==180){
        Climber.climbDown();
    }else{
        Climber.climbZero();
    }


}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
