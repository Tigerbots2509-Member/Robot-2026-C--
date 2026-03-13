#include <ctre/Phoenix.h>
#include "frc/DigitalInput.h"
#include "frc2/command/CommandPtr.h"
class climber{
    public:
        climber();
        void climbUp();
        void climbDown();
        void climbZero();
        frc::DigitalInput minClimb{5};
    private:
        ctre::phoenix::motorcontrol::can::TalonSRX mClimb{9};
};