/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
    void End(bool interrupted) override;
    bool IsFinished() override;
private:
    AutoSubsystemWrapper& m_wrapper;
    AutoCommandInfo m_info;

    bool m_field_relative;
    double m_elapsed_time = 0;
    units::meters_per_second_t m_speed_x, m_speed_y;
    double m_move_time_x = 0, m_move_time_y = 0;
    double m_x_dir = 0, m_y_dir = 0;
};
