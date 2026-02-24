// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot() :driverController{0}{}

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
    double strafing;
    double lateral;
    double distance;//value to tell you how far your camera is from the april tag
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
            frc::SmartDashboard::PutNumber("Distance PID Result:", distTargetingPID.Calculate(distance,1));
            m_container.aimedDrive.RotationalRate = rotation;
            m_container.aimedDrive.VelocityX = strafing;
            m_container.aimedDrive.VelocityY = lateral;
            m_container.drivetrain.ApplyRequest(aimedDrive);
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
