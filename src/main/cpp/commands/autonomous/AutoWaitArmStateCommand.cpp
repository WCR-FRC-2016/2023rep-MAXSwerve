#include "autonomous/commands/AutoWaitArmStateCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>

#include "utils/JsonUtils.hpp"

AutoWaitArmStateCommand::AutoWaitArmStateCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) { 
    AddRequirements(&m_wrapper.m_drive);
    AddRequirements(&m_wrapper.m_arm);
}
void AutoWaitArmStateCommand::Initialize() { }
void AutoWaitArmStateCommand::Execute() { }
void AutoWaitArmStateCommand::End(bool interrupted) { }
bool AutoWaitArmStateCommand::IsFinished() {
    return m_wrapper.m_arm.GetGoalState() == -1;
}