#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "autonomous/AutoInterpreter.hpp"
#include "autonomous/AutoCommandInfo.hpp"
#include "vendor/json.hpp"

// template<typename T>
// class AutoCommand : public frc2::CommandHelper<frc2::CommandBase, T> {
//     AutoCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& command_info) : m_wrapper(wrapper), m_command_info(command_info) { }

// protected:
//     AutoSubsystemWrapper& m_wrapper;
//     AutoCommandInfo& m_command_info;
//     //std::map<void*>
// }