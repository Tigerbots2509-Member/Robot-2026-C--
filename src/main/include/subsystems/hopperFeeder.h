#include <ctre/phoenix6/TalonFX.hpp>
class hopperFeeder{
    public:
        hopperFeeder();
        void hopperToLauncher();
        void hopperZero();
        void hopperBack();//This method is not need right now to my knowledge but in case we need it i made it
    private:
        ctre::phoenix6::hardware::TalonFX mFeeder{13};
        ctre::phoenix6::hardware::TalonFX mHopper{14};
};