#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <rev/SparkMax.h>

class CoralArm : public frc2::SubsystemBase {
    public:
        CoralArm();

        void Periodic() override;

        // Test command
        frc2::CommandPtr CoralArmRun();
            

    private:
    rev::spark::SparkMax motorL {1, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax motorR {2, rev::spark::SparkLowLevel::MotorType::kBrushless};
};