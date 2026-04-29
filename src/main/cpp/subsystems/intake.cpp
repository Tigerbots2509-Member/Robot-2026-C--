#include <subsystems/intake.h>
intake::intake(){
    frc::SmartDashboard::PutBoolean("intake lift",liftMin.Get());
    mLiftA.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    mLiftB.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
};
void intake::intakeIn(){
    mIntakeA.Set(1);
    mIntakeB.Set(-1);
    return;
};
void intake::intakeOut(){
    mIntakeA.Set(-1);
    mIntakeB.Set(1);
    return;
};
void intake::intakeStop(){
    mIntakeA.Set(0);
    mIntakeB.Set(0);
    return;
};
void intake::intakeLiftDown(){
    mLiftA.Set(0.25);
    mLiftB.Set(-0.25);
    return;
};
void intake::intakeLiftUp(){
    mLiftA.Set(-0.25);
    mLiftB.Set(0.25);
    return;
};
void intake::intakeLiftStop(){
    mLiftA.Set(0);
    mLiftB.Set(0);
    return;
};
