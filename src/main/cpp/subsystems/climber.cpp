#include <subsystems/climber.h>
climber::climber(){};
void climber::climbDown(){
    if(!minClimb.Get()){
        mClimb.Set(ControlMode::PercentOutput,-0.2);
    }else{
        climbZero();
        return;
    }
};
void climber::climbUp(){
    mClimb.Set(ControlMode::PercentOutput,0.2);
    return;
};
void climber::climbZero(){
    mClimb.Set(ControlMode::PercentOutput,0);
    return;
};