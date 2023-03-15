// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/time.h>

#include <utility>

#include "Constants.h"
#include "Logging.hpp"
#include "commands/AutoAlignCommand.h"
#include "commands/ReflectiveAlignCommand.h"
#include "commands/MoveOverCommand.h"
#include "subsystems/DriveSubsystem.h"
#include "autonomous/commands/AutoTimedMoveCommand.hpp"
#include "autonomous/commands/AutoArmMoveCommand.hpp"
#include "autonomous/commands/AutoMoveDistanceCommand.hpp"
#include "autonomous/commands/AutoClawWheelCommand.hpp"
#include "autonomous/commands/AutoWaitArmStateCommand.hpp"
#include "utils/JsonUtils.hpp"

using namespace DriveConstants;

RobotContainer::RobotContainer() : m_wrapper(m_drive, m_arm, m_limelight, m_leds) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), IOConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), IOConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), IOConstants::kDriveDeadband)},
            m_relative, m_rate_limit);
        },
        {&m_drive}));

  m_arm.SetDefaultCommand(frc2::RunCommand(
      [this] {
        // Arm
        if (m_arm.GetGoalState()==-2) {
            m_arm.Drive(
                frc::ApplyDeadband(m_manipController.GetRightY(), IOConstants::kDriveDeadband),
                frc::ApplyDeadband(-m_manipController.GetLeftY(), IOConstants::kDriveDeadband)
            );
            //Logger::Log(LogLevel::Dev) << "Arm raw lower angle: " << m_arm.GetRawLowerAngle() << LoggerCommand::Flush;
            Logger::Log(LogLevel::Dev) << "Arm lower angle: " << m_arm.GetLowerAngle() << "\n";
            Logger::Log(LogLevel::Dev) << "Arm upper angle: " << m_arm.GetUpperAngle() << "\n" << LoggerCommand::Flush;
        }

        // Claw
        auto close_pressed = m_manipController.GetLeftBumper() ?  1.0 : 0.0;
        auto open_pressed =  m_manipController.GetRightBumper() ? 1.0 : 0.0;

        auto spit = m_manipController.GetLeftTriggerAxis()  > 0.5 ? 1.0 : 0.0;
        auto suck = m_manipController.GetRightTriggerAxis() > 0.5 ? 1.0 : 0.0;

        m_arm.DriveClaw(open_pressed - close_pressed);
        m_arm.DriveCollectWheels(suck - spit);
      },
      {&m_arm}));

  m_limelight.Deactivate();
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController, ControlConstants::xModeButton)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

  // Temporary Commands

  // Toggle Field Relative
  frc2::JoystickButton(&m_driverController, ControlConstants::RelativeButton)
      .OnTrue(
          new frc2::InstantCommand([this] { m_relative ^= true; }, {&m_drive}));

  frc2::JoystickButton(&m_driverController, ControlConstants::AlignATButton)
      .WhileTrue(new AutoAlignCommand(m_drive, m_limelight));
      
  frc2::JoystickButton(&m_driverController, ControlConstants::AlignRTButton)
      .WhileTrue(new ReflectiveAlignCommand(m_drive, m_limelight));

  /*frc2::Trigger([this] {return m_driverController.GetLeftTriggerAxis()>0.5;})
      .OnTrue(frc2::ConditionalCommand(
        new MoveOverCommand(m_drive, -DriveConstants::kMoveOverSubTime),
        new MoveOverCommand(m_drive, -DriveConstants::kMoveOverTime),
        m_limelight.IsSubstation);

  frc2::Trigger([this] {return m_driverController.GetRightTriggerAxis()>0.5;})
      .OnTrue(frc2::ConditionalCommand(
        new MoveOverCommand(m_drive, DriveConstants::kMoveOverSubTime),
        new MoveOverCommand(m_drive, DriveConstants::kMoveOverTime),
        m_limelight.IsSubstation);*/

  frc2::JoystickButton(&m_driverController, ControlConstants::SwapSpeedButton)
      .OnTrue(new frc2::InstantCommand(
          [this] {
            if (m_low_speed)
              m_drive.SetSpeed(DriveConstants::kFastSpeed);
            else
              m_drive.SetSpeed(DriveConstants::kLowSpeed);
            m_low_speed ^= true;
          },
          {&m_drive}));

  // Map this to potentially reset just the rotation and not the position???
  frc2::JoystickButton(&m_driverController,
                       ControlConstants::ResetHeadingButton)
      .OnTrue(new frc2::InstantCommand([this] { m_drive.ZeroHeading(); },
                                       {&m_drive}));

  // LED State Change Command
  frc2::JoystickButton(&m_driverController, ControlConstants::DebugLEDButton)
      .OnTrue(new frc2::InstantCommand(
          [this] { m_leds.SetState((m_leds.GetState() + 1) % 3); }, {&m_leds}));

  // Arm State Change Command
  frc2::JoystickButton(&m_manipController, ControlConstants::PosCarryButton)
      .OnTrue(
          new frc2::InstantCommand([this] { m_arm.SetState(1); }, {&m_arm}));

  frc2::JoystickButton(&m_manipController, ControlConstants::PosMedButton)
      .OnTrue(
          new frc2::InstantCommand([this] { m_arm.SetState(2); }, {&m_arm}));

  frc2::JoystickButton(&m_manipController, ControlConstants::PosHighButton)
      .OnTrue(
          new frc2::InstantCommand([this] { m_arm.SetState(3); }, {&m_arm}));

  frc2::JoystickButton(&m_manipController, ControlConstants::PosSubButton)
      .OnTrue(
          new frc2::InstantCommand([this] { m_arm.SetState(4); }, {&m_arm}));

  frc2::JoystickButton(&m_manipController, ControlConstants::PosManualButton)
      .OnTrue(
          new frc2::InstantCommand([this] { m_arm.SetState(-2); }, {&m_arm}));

  frc2::JoystickButton(&m_manipController, ControlConstants::PosZeroButton)
      .OnTrue(
          new frc2::InstantCommand([this] { m_arm.SetState(6); }, {&m_arm}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    auto selected_command = AutoConstants::kAutoSequences[AutoConstants::kSelectedAuto].Commands[m_auto_command_index];
    m_auto_command_index++;

    switch(selected_command.CommandType) {
        case 0:
            return new AutoTimedMoveCommand(m_wrapper, selected_command);
        case 1:
            return new AutoArmMoveCommand(m_wrapper, selected_command);
        case 2:
            return new frc2::InstantCommand([this, selected_command] { frc::Wait(units::second_t{getValueOrDefault<double>(selected_command.CommandData, "time", 0.0)}); }, {});
        case 3:
            return new AutoMoveDistanceCommand(m_wrapper, selected_command);
        case 4:
            return new AutoClawWheelCommand(m_wrapper, selected_command);
        case 5:
            return new AutoWaitArmStateCommand(m_wrapper, selected_command);
        default:
            return new frc2::InstantCommand([this, selected_command]() { Logger::Log(LogLevel::Dev) << "Command: [" << std::to_string(selected_command.CommandType) << "] not implemented!!!" << LoggerCommand::Flush; }, {});
    }
}

void RobotContainer::ResetArmState() {
    m_arm.SetState(-1);
}

void RobotContainer::ResetAutoCommandCount() {
    m_auto_command_index = 0;
}