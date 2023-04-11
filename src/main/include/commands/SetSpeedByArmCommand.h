/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Arm.h"

#include "Logging.hpp"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SetSpeedByArmCommand
    : public frc2::CommandHelper<frc2::CommandBase, SetSpeedByArmCommand> {
 public:
  SetSpeedByArmCommand(DriveSubsystem& DriveSubsystem, Arm& arm, std::function<double()> brake, std::function<double()> turbo);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:
  DriveSubsystem& m_drive;
  Arm& m_arm;

  std::function<double()> m_brake;
  std::function<double()> m_turbo;
};
