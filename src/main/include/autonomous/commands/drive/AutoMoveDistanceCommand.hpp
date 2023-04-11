#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "autonomous/AutoCommandInfo.hpp"
#include "autonomous/AutoInterpreter.hpp"
#include "Logging.hpp"

class AutoMoveDistanceCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoMoveDistanceCommand> {
 public:
    AutoMoveDistanceCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
private:
    AutoSubsystemWrapper& m_wrapper;
    AutoCommandInfo m_info;

    bool m_field_relative;
    double m_elapsed_time = 0;
    units::meters_per_second_t m_speed_x, m_speed_y;
    units::radians_per_second_t m_speed_rot;
    double m_move_time_x = 0, m_move_time_y = 0, m_rot_time = 0;
};
