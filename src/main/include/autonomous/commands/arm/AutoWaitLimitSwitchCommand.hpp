#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "autonomous/AutoCommandInfo.hpp"
#include "autonomous/AutoInterpreter.hpp"
#include "utils/JsonUtils.hpp"
#include "Logging.hpp"

class AutoWaitLimitSwitchCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoWaitLimitSwitchCommand> {
 public:
    AutoWaitLimitSwitchCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info);

    void Initialize() override;
    bool IsFinished() override;
private:
    AutoSubsystemWrapper& m_wrapper;
    AutoCommandInfo m_info;

    int32_t m_switch;
};
