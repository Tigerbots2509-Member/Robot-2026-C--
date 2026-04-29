#include <subsystems/hopperFeeder.h>
hopperFeeder::hopperFeeder(){
    slot0Configs.kS=0.1;
    slot0Configs.kV=0.12;
    slot0Configs.kP=0.11;
    slot0Configs.kI=0;
    slot0Configs.kD=0;
    mFeeder.GetConfigurator().Apply(slot0Configs);
};

void  hopperFeeder::hopperToLauncher(){
    mFeeder.Set(-1);
    mFloor.Set(1);
    return;
};
void  hopperFeeder::hopperZero(){
    mFeeder.Set(0);
    mFloor.Set(0);
    return;
};
void  hopperFeeder::hopperBack(){
    mFeeder.Set(1);
    mFloor.Set(-1);
    return;
};