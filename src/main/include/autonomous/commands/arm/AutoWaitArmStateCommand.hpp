#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "autonomous/AutoCommandInfo.hpp"
#include "autonomous/AutoInterpreter.hpp"
#include "Logging.hpp"

class AutoWaitArmStateCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoWaitArmStateCommand> {
 public:
    AutoWaitArmStateCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info);

    bool IsFinished() override;
private:
    AutoSubsystemWrapper& m_wrapper;
    AutoCommandInfo m_info;
};
