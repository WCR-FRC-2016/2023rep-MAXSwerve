#include "autonomous/commands/AutoMoveCommand.hpp"

// AutoMoveCommand::AutoMoveCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& command_info) : AutoCommand(wrapper, command_info) { }

// AutoMoveCommand::Initialize() {
//     //m_target_x = units::meter_t{m_command_info["x-distance"].get<double>()};
//     //m_target_y = units::meter_t{m_command_info["y-distance"].get<double>()};
// }

// AutoMoveCommand::Execute() {
    

//     // auto move_x = std::clamp(m_target_x.value() - m_moved_x.value(), -1.0, 1.0);
//     // auto move_y = std::clamp(m_target_y.value() - m_moved_y.value(), -1.0, 1.0);

//     // m_wrapper.m_drive.Drive(meter_t{move_x}, meter_t{move_y}, 0_rad_per_s, false, true);

//     // m_moved_x += move_x;
//     // m_moved_y += move_y;
// }

// bool AutoMoveCommand::IsFinished() {
//     return false;//std::abs(m_moved_x) >= std::abs(m_target_x) && std::abs(m_moved_y) >= std::abs(m_target_y);
// }