#include <ctre/phoenix6/TalonFX.hpp>
#include "frc/DigitalInput.h"
#include "frc/Encoder.h"
class intake{
    public:
        intake::intake();
        void intake::intakeLiftUp();
        void intake::intakeLiftStop();
        void intake::intakeLiftDown();
        void intake::intakeIn();
        void intake::intakeStop();
        void intake::intakeOut();
    private:
        ctre::phoenix6::hardware::TalonFX mLift{15};
        ctre::phoenix6::hardware::TalonFX mIntake{10};
        frc::DigitalInput liftMax{4};
        frc::Encoder eLift{2,3};
};