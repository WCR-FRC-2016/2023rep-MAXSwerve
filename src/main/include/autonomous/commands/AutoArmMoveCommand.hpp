/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "autonomous/AutoCommandInfo.hpp"
#include "autonomous/AutoInterpreter.hpp"
#include "Logging.hpp"

class AutoArmMoveCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoArmMoveCommand> {
 public:
    AutoArmMoveCommand(AutoSubsystemWrapper& wrapper, AutoCommandInfo& info);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
private:
    AutoSubsystemWrapper& m_wrapper;
    AutoCommandInfo m_info;

    int32_t m_state;
};
