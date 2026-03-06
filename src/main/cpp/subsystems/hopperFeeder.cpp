#include <subsystems/hopperFeeder.h>
hopperFeeder::hopperFeeder(){};

frc2::CommandPtr  hopperFeeder::hopperToLauncher(){
    mFeeder.Set(0.2);
    return;
};
frc2::CommandPtr  hopperFeeder::hopperZero(){
    mFeeder.Set(0);
    return;
};
frc2::CommandPtr  hopperFeeder::hopperBack(){
    mFeeder.Set(-0.2);
    return;
};