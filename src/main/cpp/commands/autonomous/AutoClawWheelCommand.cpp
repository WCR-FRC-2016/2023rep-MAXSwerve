#include "autonomous/commands/AutoClawWheelCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>

#include "utils/JsonUtils.hpp"

AutoClawWheelCommand::AutoClawWheelCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) { 
    AddRequirements(&m_wrapper.m_drive);
    AddRequirements(&m_wrapper.m_arm);
}
void AutoClawWheelCommand::Initialize() {
    try {
        m_state = getValueOrDefault<int32_t>(m_info.CommandData, "spin-direction", -1);

        //Logger::Log(LogLevel::Dev) << "Claw Wheel Command: " << m_state << LoggerCommand::Flush;
    } catch (...) {
        Logger::Log(0b11111) << "Failed to run AutoClawWheelCommand" << LoggerCommand::Flush;
    }
}
void AutoClawWheelCommand::Execute() {
    m_wrapper.m_arm.DriveCollectWheels(m_state);
    m_elapsed_time += 0.02;
}
void AutoClawWheelCommand::End(bool interrupted) {
    m_wrapper.m_arm.DriveCollectWheels(0.0);
}
bool AutoClawWheelCommand::IsFinished() {
    return m_elapsed_time >= m_max_time;
}