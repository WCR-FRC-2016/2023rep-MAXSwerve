#include "subsystems/CTREMagEncoder.h"

CTREMagEncoder::CTREMagEncoder(int dio_slot) :
    m_encoder{dio_slot} { }

bool CTREMagEncoder::isConnected() { return m_encoder.IsConnected(); }
double CTREMagEncoder::getDistance() { return m_encoder.GetDistance(); }
units::degree_t CTREMagEncoder::getDistanceDegrees() { return units::degree_t{m_encoder.GetDistance() * 360.0}; }
