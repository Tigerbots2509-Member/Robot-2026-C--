#include "frc2/command/CommandPtr.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <subsystems/vision.h>

class launcher{
    public:
        launcher();
        void setLauncherSpeed(double distance);//This should be taking meter values from the camera
        void launchByPower();
        void wallOfBalls();
        void launchZero();
    private:
        ctre::phoenix6::configs::Slot0Configs slot0Configs{};
        double angle;
        ctre::phoenix6::hardware::TalonFX mLauncherA{11};
        ctre::phoenix6::hardware::TalonFX mLauncherB{12};
};