#include <subsystems/intake.h>
intake::intake(){
    frc::SmartDashboard::PutBoolean("intake lift",liftMin.Get());
    mLift.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
};
void intake::intakeIn(){
    if(liftMin.Get()){
        mIntake.Set(1);
        eLift.Reset();
    }else{
        intakeStop();
    };
    return;
};
void intake::intakeOut(){
    mIntake.Set(-1);
    return;
};
void intake::intakeStop(){
    mIntake.Set(0);
    return;
};
void intake::intakeLiftDown(){
    if(!liftMin.Get()){
        mLift.Set(0.15);
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
        mIntake.Set(-0.15);
    }
    return;
};
void intake::intakeLiftStop(){
    mIntake.Set(0);
    return;
};
