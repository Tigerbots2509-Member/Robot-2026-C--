#include <subsystems/intake.h>
intake::intake(){};
void intake::intakeIn(){
    if(!liftMax.Get()){
        mIntake.Set(-0.2);
    }else{
        intakeStop();
    };
    return;
};
void intake::intakeOut(){
    mIntake.Set(0.2);
    return;
};
void intake::intakeStop(){
    mIntake.Set(0);
    return;
};
void intake::intakeLiftDown(){
    if(!liftMax.Get()){
        mLift.Set(0.2);
    }else{
        eLift.Reset();
        intakeLiftStop();
    }
    return;
};
void intake::intakeLiftUp(){
    if(eLift.Get()>=700){
        intakeLiftStop();
    }else{
        mIntake.Set(-0.2);
    }
    return;
};
void intake::intakeLiftStop(){
    mIntake.Set(0);
    return;
};
