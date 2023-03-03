#pragma once

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>


class ActuatorModule {
public:
    ActuatorModule(int32_t can_id);

    void Drive(float_t dir);
private:
    rev::CANSparkMax m_max;
    //rev::SparkMaxAlternateEncoder m_encoder = m_max.GetAlternateEncoder();
};