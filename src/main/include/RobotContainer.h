// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/DriverStation.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "autonomous/AutoInterpreter.hpp"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Arm.h"
#include "subsystems/Limelight.h"
#include "subsystems/LEDController.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  void PostConfigInit();
  void InitTeleop();
  void InitAutonomous();

 private:
  // The driver's controller
  frc::XboxController m_driverController{IOConstants::kDriverControllerPort};
  frc::XboxController m_manipController{IOConstants::kManipControllerPort};
  //frc::XboxController m_debugController{IOConstants::kDebugControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  Arm m_arm;
  Limelight m_limelight;
  LEDController m_leds;
  AutoSubsystemWrapper m_wrapper;

  bool m_relative = true;
  bool m_rate_limit = true;

  // Autonomous
  int m_selected_auto;
  uint32_t m_auto_command_index = 0;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};
