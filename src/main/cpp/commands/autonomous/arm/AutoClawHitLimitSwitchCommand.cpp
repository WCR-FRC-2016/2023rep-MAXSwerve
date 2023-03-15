#include "autonomous/commands/arm/AutoClawHitLimitSwitchCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>

#include "utils/JsonUtils.hpp"

AutoClawHitLimitSwitchCommand::AutoClawHitLimitSwitchCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) { 
    int32_t state = getValueOrDefault<int32_t>(info.CommandData, "limit-switch", 3) == 3 ? 1 : -1;
    wrapper.m_arm.SetClawUseState(true);
    wrapper.m_arm.SetClawState(state);
}