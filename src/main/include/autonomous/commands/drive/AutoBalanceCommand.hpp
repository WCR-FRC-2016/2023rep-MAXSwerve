#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "autonomous/AutoCommandInfo.hpp"
#include "autonomous/AutoInterpreter.hpp"
#include "Logging.hpp"

class AutoBalanceCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoBalanceCommand> {
 public:
    AutoBalanceCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info);
    AutoBalanceCommand(AutoSubsystemWrapper& wrapper);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
private:
    AutoSubsystemWrapper& m_wrapper;
    AutoCommandInfo m_info;

    int m_old_state;
};
