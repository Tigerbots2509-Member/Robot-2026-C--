#include "frc2/command/CommandPtr.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include "frc/smartdashboard/SmartDashboard.h"
class launcher{
    public:
        launcher();
        void setLauncherSpeed(double* distance);//This should be taking meter values from the camera
        void launchByPower();
        void wallOfBalls();
        void launchZero();
    private:
        double angle;
        ctre::phoenix6::hardware::TalonFX mLauncherA{11};
        ctre::phoenix6::hardware::TalonFX mLauncherB{12};
};