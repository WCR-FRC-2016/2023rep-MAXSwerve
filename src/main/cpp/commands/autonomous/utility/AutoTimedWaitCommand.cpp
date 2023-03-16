#include "autonomous/commands/utility/AutoTimedWaitCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>

#include "utils/JsonUtils.hpp"

AutoTimedWaitCommand::AutoTimedWaitCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) { 
    AddRequirements(&m_wrapper.m_limelight);
    AddRequirements(&m_wrapper.m_drive);
    AddRequirements(&m_wrapper.m_arm);
}
void AutoTimedWaitCommand::Initialize() {
    try {
        m_target_time = getValueOrDefault<double>(m_info.CommandData, "time", 0.0);
    } catch (...) {
        Logger::Log(LogLevel::Autonomous) << "Failed to run AutoTimedWaitCommand" << LoggerCommand::Flush;
    }
}
void AutoTimedWaitCommand::Execute() {
    m_elapsed_time += 0.02;
}
void AutoTimedWaitCommand::End(bool interrupted) {

}
bool AutoTimedWaitCommand::IsFinished() {
    return m_elapsed_time > m_target_time;
}