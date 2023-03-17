#include "autonomous/commands/drive/AutoMoveDistanceCommand.hpp"

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>

#include "utils/JsonUtils.hpp"

AutoMoveDistanceCommand::AutoMoveDistanceCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) : m_wrapper(wrapper), m_info(info) { 
    AddRequirements(&m_wrapper.m_drive);
    AddRequirements(&m_wrapper.m_arm);
}
void AutoMoveDistanceCommand::Initialize() {
    try {
        auto x_dist   = getValueOrDefault<double>(m_info.CommandData, "distance-x", 0.0);
        auto x_speed = getValueOrDefault<double>(m_info.CommandData, "speed-x", 0.0);

        auto y_dist   = getValueOrDefault<double>(m_info.CommandData, "distance-y", 0.0);
        auto y_speed = getValueOrDefault<double>(m_info.CommandData, "speed-y", 0.0);

        auto rot_dist = getValueOrDefault<double>(m_info.CommandData, "distance-rot", 0.0);
        auto rot_speed_double = getValueOrDefault<double>(m_info.CommandData, "speed-rot", 0.0);
        auto max_rot_speed_double = (DriveConstants::kMaxAngularSpeed.value() * 180.0) / M_PI;

        m_field_relative = getValueOrDefault<bool>(m_info.CommandData, "field-relative", false);

        if (std::abs(x_speed)   > DriveConstants::kMaxSpeed.value()) x_speed = (x_speed / std::abs(x_speed)) * DriveConstants::kMaxSpeed.value();
        if (std::abs(y_speed)   > DriveConstants::kMaxSpeed.value()) y_speed = (y_speed / std::abs(y_speed)) * DriveConstants::kMaxSpeed.value();
        if (std::abs(rot_speed_double) > max_rot_speed_double) rot_speed_double = (rot_speed_double / std::abs(rot_speed_double)) * max_rot_speed_double;

        m_speed_x = units::meters_per_second_t{x_speed};
        m_speed_y = units::meters_per_second_t{y_speed};
        m_speed_rot = units::radians_per_second_t{(rot_speed_double * M_PI) / 180.0};

        Logger::Log(LogLevel::Autonomous) << x_speed << " " << y_speed << " " << rot_speed_double << LoggerCommand::Flush;
        //Logger::Log(LogLevel::Dev) << 

        if (x_speed == 0.0) 
            m_move_time_x = 0.0;
        else
            m_move_time_x = std::abs(x_dist / x_speed);
        if (y_speed == 0.0) 
            m_move_time_y = 0.0;
        else
            m_move_time_y = std::abs(y_dist / y_speed);
        if (rot_speed_double == 0.0) 
            m_rot_time = 0.0;
        else
            m_rot_time = std::abs(rot_dist / rot_speed_double);

        Logger::Log(LogLevel::Autonomous) << "Speed-x: " << m_speed_x << ", Speed-y: " << m_speed_y << ", Speed-rot" << m_speed_rot << LoggerCommand::Flush; 
        Logger::Log(LogLevel::Autonomous) << "Time-x: " << m_move_time_x << ", Time-y: " << m_move_time_y << ", Time-rot" << m_rot_time << LoggerCommand::Flush; 
    } catch (...) {
        Logger::Log(LogLevel::Autonomous) << "Failed to initialize AutoMoveDistanceCommand" << LoggerCommand::Flush;
    }
    
}
void AutoMoveDistanceCommand::Execute() {
    units::meters_per_second_t x_move = 0_mps, y_move = 0_mps;
    units::radians_per_second_t rot = 0_rad_per_s;
    
    if (m_elapsed_time < m_move_time_x) x_move = m_speed_x;
    if (m_elapsed_time < m_move_time_y) y_move = m_speed_y;
    if (m_elapsed_time < m_rot_time) rot = m_speed_rot;

    m_wrapper.m_drive.Drive(x_move, y_move, rot, false, m_field_relative);

    m_elapsed_time += 0.02;
}
bool AutoMoveDistanceCommand::IsFinished() {
    return (m_elapsed_time > std::max(m_rot_time, std::max(m_move_time_x, m_move_time_y)));
}