#include <subsystems/hopperFeeder.h>
hopperFeeder::hopperFeeder(){};

void  hopperFeeder::hopperToLauncher(){
    mFeeder.Set(0.8);
    return;
};
void  hopperFeeder::hopperZero(){
    mFeeder.Set(0);
    return;
};
void  hopperFeeder::hopperBack(){
    mFeeder.Set(-0.8);
    return;
};