#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "autonomous/AutoCommandInfo.hpp"
#include "autonomous/AutoInterpreter.hpp"
#include "Logging.hpp"

class AutoArmMoveCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoArmMoveCommand> {
 public:
    AutoArmMoveCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info);

    bool IsFinished() override;
private:
    AutoSubsystemWrapper& m_wrapper;
    AutoCommandInfo m_info;

    int32_t m_state;
    bool m_wait;
};
