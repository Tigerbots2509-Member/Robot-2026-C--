#include <ctre/Phoenix.h>
#include "frc/DigitalInput.h"
class climber{
    public:
        climber();
        void climbUp();
        void climbDown();
        void climbZero();
    private:
        frc::DigitalInput minClimb{5};
        ctre::phoenix::motorcontrol::can::TalonSRX mClimb{9};
    };