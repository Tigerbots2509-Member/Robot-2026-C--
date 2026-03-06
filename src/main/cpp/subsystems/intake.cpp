#include <subsystems/intake.h>
intake::intake(){};
frc2::CommandPtr intake::intakeIn(){
    if(!liftMax.Get()){
        mIntake.Set(-0.2);
    }else{
        intakeStop();
    };
    return;
};
frc2::CommandPtr intake::intakeOut(){
    mIntake.Set(0.2);
    return;
};
frc2::CommandPtr intake::intakeStop(){
    mIntake.Set(0);
    return;
};
frc2::CommandPtr intake::intakeLiftDown(){
    if(!liftMax.Get()){
        mLift.Set(0.2);
    }else{
        eLift.Reset();
        intakeLiftStop();
    }
    return;
};
frc2::CommandPtr intake::intakeLiftUp(){
    if(eLift.Get()>=700){
        intakeLiftStop();
    }else{
        mIntake.Set(-0.2);
    }
    return;
};
frc2::CommandPtr intake::intakeLiftStop(){
    mIntake.Set(0);
    return;
};
