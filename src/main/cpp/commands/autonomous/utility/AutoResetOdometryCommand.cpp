#include "autonomous/commands/utility/AutoResetOdometryCommand.hpp"

AutoResetOdometryCommand::AutoResetOdometryCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info) {
    wrapper.m_drive.ResetOdometry(frc::Pose2d{});
}