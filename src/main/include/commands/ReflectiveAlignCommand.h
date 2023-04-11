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
#include "subsystems/Limelight.h"

#include "Logging.hpp"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ReflectiveAlignCommand
    : public frc2::CommandHelper<frc2::CommandBase, ReflectiveAlignCommand> {
 public:
  ReflectiveAlignCommand(DriveSubsystem& DriveSubsystem, Limelight& limelight);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
private:
  DriveSubsystem& m_drive;
  Limelight& m_limelight;

  template <typename T> T Abs(T x);
};
