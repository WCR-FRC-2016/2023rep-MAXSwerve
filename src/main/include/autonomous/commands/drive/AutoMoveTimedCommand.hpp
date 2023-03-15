#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <units/velocity.h>
#include <units/angle.h>

#include "autonomous/AutoCommandInfo.hpp"
#include "autonomous/AutoInterpreter.hpp"
#include "utils/JsonUtils.hpp"
#include "Logging.hpp"

class AutoMoveTimedCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoMoveTimedCommand> {
 public:
    AutoMoveTimedCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
private:
    AutoSubsystemWrapper& m_wrapper;
    AutoCommandInfo m_info;

    units::meters_per_second_t m_x_speed = 0_mps, m_y_speed = 0_mps;
    units::radians_per_second_t m_rot_speed = 0_rad_per_s;
    double m_x_time = 0.0, m_y_time = 0.0, m_rot_time = 0.0;
    double m_elapsed_time = 0.00;
    bool m_field_relative = false;
};
