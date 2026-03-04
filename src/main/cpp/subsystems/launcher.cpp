#include <subsystems/launcher.h>

launcher::launcher(){};
void launcher::launchByDist(double distance){
    double power=distance*.123;//Change this for a formula that works
    mLauncherA.Set(power);
    mLauncherB.Set(-1*power);
    return;
};
void launcher::launchByPower(){
    mLauncherA.Set(0.5);
    mLauncherB.Set(-0.5);
    return;
};
void launcher::wallOfBalls(){
    mLauncherA.Set(0.9);
    mLauncherB.Set(-0.9);
    return;
};
void launcher::launchZero(){
    mLauncherA.Set(0);
    mLauncherB.Set(0);
    return;
}