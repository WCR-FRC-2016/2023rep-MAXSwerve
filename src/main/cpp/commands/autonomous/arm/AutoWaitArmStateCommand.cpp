#include "autonomous/commands/arm/AutoWaitArmStateCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>

#include "utils/JsonUtils.hpp"

AutoWaitArmStateCommand::AutoWaitArmStateCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) { 
    AddRequirements(&m_wrapper.m_arm);
}
bool AutoWaitArmStateCommand::IsFinished() {
    Logger::Log(LogLevel::Autonomous) << "Is Finished: " << (m_wrapper.m_arm.GetGoalState() == -1) << LoggerCommand::Flush;
    Logger::Log(LogLevel::Autonomous) << "State: " << std::to_string(m_wrapper.m_arm.GetGoalState()) << LoggerCommand::Flush;

    return m_wrapper.m_arm.GetGoalState() == -1;
}