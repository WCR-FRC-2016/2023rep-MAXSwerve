#include "autonomous/commands/arm/AutoArmMoveCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>

#include "utils/JsonUtils.hpp"

AutoArmMoveCommand::AutoArmMoveCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) { 
    AddRequirements({&m_wrapper.m_drive, &m_wrapper.m_arm});

    m_state = getValueOrDefault<int32_t>(info.CommandData, "arm-state", 4);
    m_wait  = getValueOrDefault<bool>(info.CommandData, "wait", false);

    m_wrapper.m_arm.SetState(m_state);
}
bool AutoArmMoveCommand::IsFinished() {
    return true;
}