#include <ctre/Phoenix.h>
#include "frc/DigitalInput.h"
#include "frc2/command/CommandPtr.h"
class climber{
    public:
        climber();
        frc2::CommandPtr climbUp();
        frc2::CommandPtr climbDown();
        frc2::CommandPtr climbZero();
    private:
        frc::DigitalInput minClimb{5};
        ctre::phoenix::motorcontrol::can::TalonSRX mClimb{9};
};