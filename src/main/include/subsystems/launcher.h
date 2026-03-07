#include "frc2/command/CommandPtr.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include "frc/smartdashboard/SmartDashboard.h"
class launcher{
    public:
        launcher();
        frc2::CommandPtr setLauncherSpeed(double targetSpeed);
        frc2::CommandPtr launchByPower();
        frc2::CommandPtr wallOfBalls();
        frc2::CommandPtr launchZero();
    private:
        ctre::phoenix6::hardware::TalonFX mLauncherA{11};
        ctre::phoenix6::hardware::TalonFX mLauncherB{12};
};