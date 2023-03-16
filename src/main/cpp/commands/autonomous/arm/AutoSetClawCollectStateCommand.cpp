#include "autonomous/commands/arm/AutoSetClawCollectStateCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>

#include "utils/JsonUtils.hpp"

AutoSetClawCollectStateCommand::AutoSetClawCollectStateCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) { 
    wrapper.m_arm.SetCollectState(getValueOrDefault<int32_t>(info.CommandData, "claw-state", 0));
}