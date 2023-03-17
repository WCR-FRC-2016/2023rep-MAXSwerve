#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <units/velocity.h>
#include <units/angle.h>

#include "autonomous/AutoCommandInfo.hpp"
#include "autonomous/AutoInterpreter.hpp"
#include "utils/JsonUtils.hpp"
#include "Logging.hpp"

class AutoDriveClawCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoDriveClawCommand> {
 public:
    AutoDriveClawCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
private:
    AutoSubsystemWrapper& m_wrapper;
    AutoCommandInfo m_info;

    double m_state;
    double m_elapsed_time = 0.00;
};
