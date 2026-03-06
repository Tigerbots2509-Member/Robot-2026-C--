#include <subsystems/climber.h>
climber::climber(){};
frc2::CommandPtr climber::climbDown(){
    if(!minClimb.Get()){
        mClimb.Set(ControlMode::PercentOutput,-0.2);
    }else{
        climbZero();
        return;
    }
};
frc2::CommandPtr climber::climbUp(){
    mClimb.Set(ControlMode::PercentOutput,0.2);
    return;
};
frc2::CommandPtr climber::climbZero(){
    mClimb.Set(ControlMode::PercentOutput,0);
    return;
};