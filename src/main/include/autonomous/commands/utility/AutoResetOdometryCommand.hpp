#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>

#include "autonomous/AutoCommandInfo.hpp"
#include "autonomous/AutoInterpreter.hpp"
#include "Logging.hpp"

class AutoResetOdometryCommand
    : public frc2::InstantCommand {
 public:
    AutoResetOdometryCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info);
};
