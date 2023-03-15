#include "autonomous/commands/arm/AutoWaitLimitSwitchCommand.hpp"

AutoWaitLimitSwitchCommand::AutoWaitLimitSwitchCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) { }

void AutoWaitLimitSwitchCommand::Initialize() {
    try {
        m_switch = getValueOrDefault(m_info.CommandData, "limit-switch", 3);
    } catch(...) {
        Logger::Log(LogLevel::Autonomous) << "Failed to Initialize AutoWaitLimitSwitchCommand" << LoggerCommand::Flush;
    }
}

bool AutoWaitLimitSwitchCommand::IsFinished() {
    switch (m_switch) {
        case 3:
            return m_wrapper.m_arm.GetOuterLimitSwitchState();
        case 4:
            return m_wrapper.m_arm.GetInnerLimitSwitchState();
        default:
            Logger::Log(LogLevel::Autonomous) << "Unknown limit switch: [id = " << m_switch << "]" << LoggerCommand::Flush;
            return true; // Unknown limit switch, return true to prevent things from breaking
    }
}