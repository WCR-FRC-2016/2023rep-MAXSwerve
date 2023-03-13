#include "autonomous/commands/AutoArmMoveCommand.hpp"

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

        Logger::Log(LogLevel::Dev) << "Move Arm Command: " << m_state << LoggerCommand::Flush;
    } catch (...) {
        Logger::Log(0b11111) << "Failed to run AutoArmMoveCommand" << LoggerCommand::Flush;
    }
}
void AutoArmMoveCommand::Execute() {
    if (m_state <= -1) return;

    m_wrapper.m_arm.SetState(m_state);
    m_state = -1;
}
void AutoArmMoveCommand::End(bool interrupted) {

}
bool AutoArmMoveCommand::IsFinished() {
    return m_state <= -1; // Should always be -1 if ran
}