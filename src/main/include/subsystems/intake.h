#include <ctre/phoenix6/TalonFX.hpp>
#include "frc/DigitalInput.h"
#include "frc/Encoder.h"
class intake{
    public:
        intake();
        void intakeLiftUp();
        void intakeLiftStop();
        void intakeLiftDown();
        void intakeIn();
        void intakeStop();
        void intakeOut();
    private:
        ctre::phoenix6::hardware::TalonFX mLift{15};
        ctre::phoenix6::hardware::TalonFX mIntake{10};
        frc::DigitalInput liftMax{4};
        frc::Encoder eLift{2,3};
};