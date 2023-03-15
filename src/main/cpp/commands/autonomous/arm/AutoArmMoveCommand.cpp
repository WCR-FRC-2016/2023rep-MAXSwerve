#include "autonomous/commands/arm/AutoArmMoveCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>

#include "utils/JsonUtils.hpp"

AutoArmMoveCommand::AutoArmMoveCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) { 
    AddRequirements(&m_wrapper.m_drive);
    AddRequirements(&m_wrapper.m_arm);
}
void AutoArmMoveCommand::Initialize() {
    try {
        m_state = getValueOrDefault<int32_t>(m_info.CommandData, "arm-state", -1);
        m_wait  = getValueOrDefault<bool>(m_info.CommandData, "wait", false);
    } catch (...) {
        Logger::Log(LogLevel::Autonomous) << "Failed to initialize AutoArmMoveCommand" << LoggerCommand::Flush;
    }
}
void AutoArmMoveCommand::Execute() {
    if (m_state <= -1) return;

    m_wrapper.m_arm.SetState(m_state);
    m_state = -1;
}
bool AutoArmMoveCommand::IsFinished() {
    return m_wait ? (m_state <= -1) : (m_wrapper.m_arm.GetGoalState() == -1);
}