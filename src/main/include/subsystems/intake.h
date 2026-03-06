#include <ctre/phoenix6/TalonFX.hpp>
#include "frc/DigitalInput.h"
#include "frc2/command/CommandPtr.h"
#include "frc/Encoder.h"
class intake{
    public:
        intake();
        frc2::CommandPtr intakeLiftUp();
        frc2::CommandPtr intakeLiftStop();
        frc2::CommandPtr intakeLiftDown();
        frc2::CommandPtr intakeIn();
        frc2::CommandPtr intakeStop();
        frc2::CommandPtr intakeOut();
    private:
        ctre::phoenix6::hardware::TalonFX mLift{15};
        ctre::phoenix6::hardware::TalonFX mIntake{10};
        frc::DigitalInput liftMax{4};
        frc::Encoder eLift{2,3};
};