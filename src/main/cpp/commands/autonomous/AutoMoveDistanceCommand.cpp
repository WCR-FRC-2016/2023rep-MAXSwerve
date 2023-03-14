#include "autonomous/commands/AutoMoveDistanceCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>

#include "utils/JsonUtils.hpp"

AutoMoveDistanceCommand::AutoMoveDistanceCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) { 
    AddRequirements(&m_wrapper.m_drive);
    AddRequirements(&m_wrapper.m_arm);
}
void AutoMoveDistanceCommand::Initialize() {
    try {
        const auto x_dist = getValueOrDefault<double>(m_info.CommandData, "distance-x", 0.0);
        const auto y_dist = getValueOrDefault<double>(m_info.CommandData, "distance-y", 0.0);

        auto x_speed = getValueOrDefault<double>(m_info.CommandData, "speed-x", 0.0);
        auto y_speed = getValueOrDefault<double>(m_info.CommandData, "speed-y", 0.0);

        m_field_relative = getValueOrDefault<bool>(m_info.CommandData, "field-relative", false);

        if (x_speed > DriveConstants::kMaxSpeed.value()) x_speed = DriveConstants::kMaxSpeed.value();
        if (y_speed > DriveConstants::kMaxSpeed.value()) y_speed = DriveConstants::kMaxSpeed.value();

        m_speed_x = x_speed / DriveConstants::kMaxSpeed;
        m_speed_y = y_speed / DriveConstants::kMaxSpeed;

        m_move_time_x = x_dist / x_speed;
        m_move_time_y = y_dist / y_speed;

        m_x_dir = (x_speed >= 0) ? 1.0 : -1.0;
        m_y_dir = (y_speed >= 0) ? 1.0 : -1.0;
    } catch (...) {
        Logger::Log(0b11111) << "Failed to run AutoMoveDistanceCommand" << LoggerCommand::Flush;
    }
    
}
void AutoMoveDistanceCommand::Execute() {
    units::meters_per_second_t x_move = 0_mps, y_move = 0_mps;
    
    if (m_elapsed_time < m_move_time_x) x_move = m_speed_x * m_x_dir;
    if (m_elapsed_time < m_move_time_y) y_move = m_speed_y * m_y_dir;

    m_wrapper.m_drive.Drive(x_move, y_move, 0.0_rad_per_s, false, false);

    m_elapsed_time += 0.02;
}
void AutoMoveDistanceCommand::End(bool interrupted) {

}
bool AutoMoveDistanceCommand::IsFinished() {
    return (m_elapsed_time >= std::max(m_move_time_x, m_move_time_y));
}