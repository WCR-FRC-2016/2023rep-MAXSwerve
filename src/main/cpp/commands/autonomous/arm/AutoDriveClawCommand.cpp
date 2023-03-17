#include "autonomous/commands/arm/AutoDriveClawCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>

#include "utils/JsonUtils.hpp"

AutoDriveClawCommand::AutoDriveClawCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) { 
    AddRequirements({&m_wrapper.m_drive, &m_wrapper.m_arm});
}
void AutoDriveClawCommand::Initialize() {
    try {
        m_state = getValueOrDefault<double>(m_info.CommandData, "claw-state", 0.0);
    } catch (...) {
        Logger::Log(LogLevel::Autonomous) << "Failed to initialize AutoDriveClawCommand" << LoggerCommand::Flush;
    }
    
}
void AutoDriveClawCommand::Execute() {
    m_wrapper.m_arm.DriveCollectWheels(m_state);

    m_elapsed_time += 0.02;
}
bool AutoDriveClawCommand::IsFinished() {
    return (m_elapsed_time > 2.0);
}