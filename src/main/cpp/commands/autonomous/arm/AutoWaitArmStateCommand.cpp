#include "autonomous/commands/arm/AutoWaitArmStateCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>

#include "utils/JsonUtils.hpp"

AutoWaitArmStateCommand::AutoWaitArmStateCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) { 
    AddRequirements({&m_wrapper.m_drive,&m_wrapper.m_arm});
}
bool AutoWaitArmStateCommand::IsFinished() {
    return m_wrapper.m_arm.GetGoalState() == -1;
}