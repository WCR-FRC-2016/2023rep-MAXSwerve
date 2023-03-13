#include "autonomous/commands/AutoTimedMoveCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>

#include "utils/JsonUtils.hpp"

AutoTimedMoveCommand::AutoTimedMoveCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) { 
    AddRequirements(&m_wrapper.m_drive);
    AddRequirements(&m_wrapper.m_arm);
}
void AutoTimedMoveCommand::Initialize() {
    try {
        //Logger::Log(0b1111111) << "INITIALIZING COMMAND" << LoggerCommand::Flush;
        //Logger::Log(LogLevel::Dev) << m_info.CommandData.dump() << LoggerCommand::Flush;

        m_move_time_x = getValueOrDefault<double>(m_info.CommandData, "time-x", 0.0);
        m_move_time_y = getValueOrDefault<double>(m_info.CommandData, "time-y", 0.0);
        m_x_dir = getValueOrDefault<double>(m_info.CommandData, "dir-x", 0.0);
        m_y_dir = getValueOrDefault<double>(m_info.CommandData, "dir-y", 0.0);

        Logger::Log(LogLevel::Dev) << "Timed Move Command: " << m_move_time_x << " : " << m_move_time_y << " : " << m_x_dir << " : " << m_y_dir << LoggerCommand::Flush;
    } catch (...) {
        Logger::Log(0b11111) << "Failed to run AutoTimedMoveCommand" << LoggerCommand::Flush;
    }
    
    //m_move_time_x = m_info.CommandData["time-x"].get<double>();
    //m_move_time_y = m_info.CommandData["time-y"].get<double>();
    //m_x_dir = m_info.CommandData["dir-x"].get<double>();
    //m_y_dir = m_info.CommandData["dir-y"].get<double>();
}
void AutoTimedMoveCommand::Execute() {
    units::meters_per_second_t x_move = 0_mps, y_move = 0_mps;
    
    if (m_elapsed_time < m_move_time_x) x_move = 1_mps * m_x_dir;
    if (m_elapsed_time < m_move_time_y) y_move = 1_mps * m_y_dir;

    m_wrapper.m_drive.Drive(x_move, y_move, 0.0_rad_per_s, false, false);

    m_elapsed_time += 0.02;
}
void AutoTimedMoveCommand::End(bool interrupted) {

}
bool AutoTimedMoveCommand::IsFinished() {
    return (m_elapsed_time >= std::max(m_move_time_x, m_move_time_y));
}