#pragma once

#include <units/angle.h>
#include <frc/DutyCycleEncoder.h>

class CTREMagEncoder {
public:
    CTREMagEncoder(int dio_slot);

    bool isConnected();
    double getDistance();
    units::degree_t getDistanceDegrees();
private:
    frc::DutyCycleEncoder m_encoder;
};