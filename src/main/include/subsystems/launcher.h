#include <ctre/phoenix6/TalonFX.hpp>
class launcher{
    public:
        launcher();
        void launchByDist(double distance);
        void launchByPower();
        void wallOfBalls();
        void launchZero();
    private:
        ctre::phoenix6::hardware::TalonFX mLauncherA{11};
        ctre::phoenix6::hardware::TalonFX mLauncherB{12};
};