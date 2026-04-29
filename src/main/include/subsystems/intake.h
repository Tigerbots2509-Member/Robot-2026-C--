#include <ctre/phoenix6/TalonFX.hpp>
#include "frc/DigitalInput.h"
#include "frc2/command/CommandPtr.h"
#include "frc/Encoder.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <ctre/phoenix6/signals/SpnEnums.hpp>
class intake{
    public:
        intake();
        void intakeLiftUp();
        void intakeLiftStop();
        void intakeLiftDown();
        void intakeIn();
        void intakeStop();
        void intakeOut();
        frc::DigitalInput liftMin{4};
        frc::Encoder eLift{2,3};
    private:
        ctre::phoenix6::hardware::TalonFX mLiftA{15};
        ctre::phoenix6::hardware::TalonFX mLiftB{16};
        ctre::phoenix6::hardware::TalonFX mIntakeA{13};
        ctre::phoenix6::hardware::TalonFX mIntakeB{14};
        
};