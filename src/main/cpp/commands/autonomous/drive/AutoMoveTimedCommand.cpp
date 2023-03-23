#include "autonomous/commands/drive/AutoMoveTimedCommand.hpp"

AutoMoveTimedCommand::AutoMoveTimedCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) {
    AddRequirements(&m_wrapper.m_drive);
}

void AutoMoveTimedCommand::Initialize() {
    try {
        m_x_time = getValueOrDefault(m_info.CommandData, "time-x", 0.0);
        m_x_speed = units::meters_per_second_t{getValueOrDefault(m_info.CommandData, "speed-x", 0.0)};

        m_y_time = getValueOrDefault(m_info.CommandData, "time-y", 0.0);
        m_x_speed = units::meters_per_second_t{getValueOrDefault(m_info.CommandData, "speed-y", 0.0)};

        m_rot_time = getValueOrDefault(m_info.CommandData, "time-rot", 0.0);
        m_rot_speed = units::radians_per_second_t{(getValueOrDefault(m_info.CommandData, "speed-rot", 0.0) * M_PI) / 180.0};

        m_field_relative = getValueOrDefault(m_info.CommandData, "field-relative", false);
    } catch(...) {
        Logger::Log(LogLevel::Autonomous) << "Failed to initialize AutoMoveTimedCommand" << LoggerCommand::Flush;
    }
}
void AutoMoveTimedCommand::Execute() {
    auto x_speed = m_x_speed;
    auto y_speed = m_y_speed;
    auto rot_speed = m_rot_speed;

    if (m_elapsed_time > m_x_time) x_speed = 0_mps;
    if (m_elapsed_time > m_y_time) y_speed = 0_mps;
    if (m_elapsed_time > m_rot_time) rot_speed = 0_rad_per_s;

    m_wrapper.m_drive.Drive(x_speed, y_speed, rot_speed, m_field_relative, false);

    m_elapsed_time = 0.02;
}
void AutoMoveTimedCommand::End(bool interrupted) {
    m_wrapper.m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
}
bool AutoMoveTimedCommand::IsFinished() {
    return m_elapsed_time > std::max(std::max(m_x_time, m_y_time), m_rot_time);
}