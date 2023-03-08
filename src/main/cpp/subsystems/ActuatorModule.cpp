#include "subsystems/ActuatorModule.h"

#include "units/current.h"

ActuatorModule::ActuatorModule(int32_t can_id) : m_max(can_id, rev::CANSparkMax::MotorType::kBrushed) { 
    m_max.RestoreFactoryDefaults();

    m_max.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_max.SetSmartCurrentLimit((10_A).value());

    m_max.BurnFlash();
};

void ActuatorModule::Drive(float_t dir) {
    m_max.Set(-dir);
    //m_encoder.SetPosition(m_encoder.GetPosition() + dir);
};