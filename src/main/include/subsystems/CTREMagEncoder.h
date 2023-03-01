#pragma once

#include <units/angle.h>
#include <frc/DutyCycleEncoder.h>

class CTREMagEncoder {
public:
    CTREMagEncoder(int dio_slot);

    bool IsConnected();
    bool IsAbove(const units::degree_t& value);
    bool IsAboveAbs(const units::degree_t& value);
    bool IsBelow(const units::degree_t& value);
    bool IsBelowAbs(const units::degree_t& value);
    bool IsBetween(const units::degree_t& a, const units::degree_t& b);
    double GetDistance();
    int32_t GetRevolutions();
    units::degree_t GetDistanceDegrees();
    units::degree_t GetNormalizedDistanceDegrees();
private:
    frc::DutyCycleEncoder m_encoder;
};