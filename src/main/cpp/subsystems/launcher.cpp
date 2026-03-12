#include <subsystems/launcher.h>
#include <subsystems/vision.h>
#include <math.h>
launcher::launcher(){
    ctre::phoenix6::configs::Slot0Configs slot0Configs{};
    slot0Configs.kS=0.1;
    slot0Configs.kV=0.12;
    slot0Configs.kP=0.11;
    slot0Configs.kI=0;
    slot0Configs.kD=0;
    mLauncherA.GetConfigurator().Apply(slot0Configs);
    mLauncherB.GetConfigurator().Apply(slot0Configs);
    frc::SmartDashboard::PutNumber("Launch",0.0);
};
void launcher::setLauncherSpeed(double* distance){//This should be taking meter values from camera
    if(rotationalValues(ClosestHubId("limelight-b"), distance, &angle,5.75,10)){  //This is here so that we can call this in the Robot Container binding Configs
        frc::SmartDashboard::GetNumber("Launch",0.0);
        double targetSpeed=((log((183.75/(*distance))-1))-6.41678)/-0.133244;
        ctre::phoenix6::controls::VelocityVoltage m_request =ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
        mLauncherA.SetControl(m_request.WithVelocity(targetSpeed*1_tps).WithFeedForward(0.5_V));
        mLauncherB.SetControl(m_request.WithVelocity(targetSpeed*1_tps).WithFeedForward(0.5_V));
    }
    return;
};
void launcher::launchByPower(){
    mLauncherA.Set(0.5);
    mLauncherB.Set(0.5);
    return;
};
void launcher::wallOfBalls(){
    mLauncherA.Set(0.9);
    mLauncherB.Set(0.9);
    return;
};
void launcher::launchZero(){
    mLauncherA.Set(0);
    mLauncherB.Set(0);
    return;
}