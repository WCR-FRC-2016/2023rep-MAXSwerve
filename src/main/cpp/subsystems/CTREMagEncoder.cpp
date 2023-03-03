#include "subsystems/CTREMagEncoder.h"

CTREMagEncoder::CTREMagEncoder(int dio_slot) :
    m_encoder{dio_slot} { }

bool CTREMagEncoder::IsConnected() { return m_encoder.IsConnected(); }

bool CTREMagEncoder::IsAbove(const units::degree_t& value) { return GetNormalizedDistanceDegrees().value() < value.value(); }
bool CTREMagEncoder::IsAboveAbs(const units::degree_t& value) { return abs(GetNormalizedDistanceDegrees().value()) < value.value(); }
bool CTREMagEncoder::IsBelow(const units::degree_t& value) { return GetNormalizedDistanceDegrees().value() < value.value(); }
bool CTREMagEncoder::IsBelowAbs(const units::degree_t& value) { return abs(GetNormalizedDistanceDegrees().value()) < value.value(); }
bool CTREMagEncoder::IsBetween(const units::degree_t& a, const units::degree_t& b) {
    auto degree = GetNormalizedDistanceDegrees().value();

    if (a.value() > b.value()) return (degree >= b.value()) && (degree <= a.value());
    else return (degree <= b.value()) && (degree >= a.value());
}

double CTREMagEncoder::GetDistance() { return m_encoder.GetDistance(); }
units::degree_t CTREMagEncoder::GetDistanceDegrees() { return units::degree_t{m_encoder.GetDistance() * 360.0}; }
units::degree_t CTREMagEncoder::GetNormalizedDistanceDegrees() {
    int turns = static_cast<int32_t>(m_encoder.GetDistance());
    return units::degree_t{GetDistanceDegrees().value() - (static_cast<double>(turns) * 360.0)};
}
int32_t CTREMagEncoder::GetRevolutions() { return static_cast<int32_t>(m_encoder.GetDistance()); }