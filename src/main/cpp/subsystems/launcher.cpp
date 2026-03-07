#include <subsystems/launcher.h>

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
frc2::CommandPtr launcher::setLauncherSpeed(double targetSpeed){
    frc::SmartDashboard::GetNumber("Launch",0.0);
    ctre::phoenix6::controls::VelocityVoltage m_request =ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
    mLauncherA.SetControl(m_request.WithVelocity(targetSpeed*1_tps).WithFeedForward(0.5_V));
    mLauncherB.SetControl(m_request.WithVelocity(targetSpeed*1_tps).WithFeedForward(0.5_V));
    return;
};
frc2::CommandPtr launcher::launchByPower(){
    mLauncherA.Set(0.5);
    mLauncherB.Set(0.5);
    return;
};
frc2::CommandPtr launcher::wallOfBalls(){
    mLauncherA.Set(0.9);
    mLauncherB.Set(0.9);
    return;
};
frc2::CommandPtr launcher::launchZero(){
    mLauncherA.Set(0);
    mLauncherB.Set(0);
    return;
}