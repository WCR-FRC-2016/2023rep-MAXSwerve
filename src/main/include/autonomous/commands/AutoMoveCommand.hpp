#pragma once

#include <units/length.h>

#include "autonomous/commands/AutoCommand.hpp"
#include "autonomous/AutoInterpreter.hpp"
#include "Logging.hpp"

// class AutoMoveCommand : public AutoCommand<AutoMoveCommand> {
// public:
//     AutoMoveCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& command_info);

//     void Initialize() override;
//     void Execute() override;
//     //void End(bool interrupted) override;
//     bool IsFinished() override;
// private:
//     units::meter_t m_target_x, m_target_y;
//     units::meter_t m_moved_x, m_moved_y;
// };
