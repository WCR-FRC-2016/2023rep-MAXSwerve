#pragma once

#include <units/angle.h>
#include <frc/DutyCycleEncoder.h>

class CTREMagEncoder {
public:
    CTREMagEncoder(int dio_slot);

    bool IsConnected();
    double GetDistance();
    units::degree_t GetDistanceDegrees();
private:
    frc::DutyCycleEncoder m_encoder;
};