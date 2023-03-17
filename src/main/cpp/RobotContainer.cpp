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
#include <frc2/command/button/POVButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/time.h>


#include <utility>

#include "Constants.h"
#include "Logging.hpp"
#include "commands/AutoAlignCommand.h"
#include "commands/ReflectiveAlignCommand.h"
#include "commands/MoveOverCommand.h"
#include "commands/MoveClawCommand.h"
#include "subsystems/DriveSubsystem.h"
#include "utils/JsonUtils.hpp"

#include "autonomous/commands/arm/AutoArmMoveCommand.hpp"
#include "autonomous/commands/arm/AutoClawHitLimitSwitchCommand.hpp"
#include "autonomous/commands/arm/AutoDriveClawCommand.hpp"
#include "autonomous/commands/arm/AutoWaitArmStateCommand.hpp"
#include "autonomous/commands/arm/AutoWaitLimitSwitchCommand.hpp"

#include "autonomous/commands/drive/AutoMoveDistanceCommand.hpp"
#include "autonomous/commands/drive/AutoMoveTimedCommand.hpp"

#include "autonomous/commands/utility/AutoResetOdometryCommand.hpp"
#include "autonomous/commands/utility/AutoTimedWaitCommand.hpp"



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
        Logger::Log(LogLevel::Dev) << "gyro: " << m_drive.GetHeading() << LoggerCommand::Flush;
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
        }

        // Claw
        //auto close_pressed = m_manipController.GetLeftBumper() ?  1.0 : 0.0;
        //auto open_pressed =  m_manipController.GetRightBumper() ? 1.0 : 0.0;

        auto spit = m_manipController.GetRightTriggerAxis()  > 0.5 ? 1.0 : 0.0;
        auto suck = m_manipController.GetLeftTriggerAxis() > 0.5 ? 1.0 : 0.0;

        //m_arm.DriveClaw(open_pressed - close_pressed);
        m_arm.DriveCollectWheels(suck - spit);

        //PrintDebugStuff();
      },
      {&m_arm}));

  m_limelight.Deactivate();
}

void RobotContainer::PrintDebugStuff() {
    Logger::Log(LogLevel::All) << "Encoder 4 state: " << TestEncoder.Get() << LoggerCommand::Flush;
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController, ControlConstants::xModeButton)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

  // Toggle Field Relative
  frc2::JoystickButton(&m_driverController, ControlConstants::RelativeButton)
      .OnTrue(
          new frc2::InstantCommand([this] { m_relative ^= true; }, {&m_drive}));

  frc2::JoystickButton(&m_driverController, ControlConstants::AlignATButton)
      .WhileTrue(new AutoAlignCommand(m_drive, m_limelight));
      
  frc2::JoystickButton(&m_driverController, ControlConstants::AlignRTButton)
      .WhileTrue(new ReflectiveAlignCommand(m_drive, m_limelight));

  frc2::JoystickButton(&m_driverController, ControlConstants::SwapSpeedButton)
      .OnTrue(new frc2::InstantCommand(
          [this] { m_drive.SwapSpeed(); },
          {&m_drive}));
  
  frc2::Trigger([this] {return m_driverController.GetLeftTriggerAxis() > 0.5;})
      .OnTrue(new frc2::ConditionalCommand(
        MoveOverCommand(m_drive, -DriveConstants::kMoveOverSubTime),
        MoveOverCommand(m_drive, -DriveConstants::kMoveOverTime),
        [this] {return m_limelight.IsSubstation();}
      ));
  
  frc2::Trigger([this] {return m_driverController.GetRightTriggerAxis() > 0.5;})
      .OnTrue(new frc2::ConditionalCommand(
        MoveOverCommand(m_drive, DriveConstants::kMoveOverSubTime),
        MoveOverCommand(m_drive, DriveConstants::kMoveOverTime),
        [this] {return m_limelight.IsSubstation();}
      ));
  

  // Map this to potentially reset just the rotation and not the position???
  frc2::JoystickButton(&m_driverController,
                       ControlConstants::ResetHeadingButton)
      .OnTrue(new frc2::InstantCommand([this] { m_drive.ZeroHeading(); },
                                       {&m_drive}));

  // LED State Change Command
  frc2::JoystickButton(&m_driverController, ControlConstants::DebugLEDButton)
      .OnTrue(new frc2::InstantCommand(
          [this] { m_leds.SetState((m_leds.GetState() + 1) % 3); }, {&m_leds}));

  // TODO: these shouldn't be backwards. What's going on?
  frc2::JoystickButton(&m_manipController,
                       ControlConstants::CloseClawButton)
      .OnTrue(new MoveClawCommand(m_arm, 1.0));
  
  frc2::JoystickButton(&m_manipController,
                       ControlConstants::OpenClawButton)
      .OnTrue(new MoveClawCommand(m_arm, -1.0));

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

    frc2::POVButton(&m_manipController, 0, 0).OnTrue(new frc2::InstantCommand([this] { 
        m_leds.SetState(1); 
        Logger::Log(LogLevel::All) << "CONE LEDS ENABLED" << LoggerCommand::Flush;
    }, {&m_leds}));
    frc2::POVButton(&m_manipController, 180, 0).OnTrue(new frc2::InstantCommand([this] { 
        m_leds.SetState(2); 
        Logger::Log(LogLevel::All) << " CUBE LEDS ENABLED" << LoggerCommand::Flush;
    }, {&m_leds}));
    frc2::POVButton(&m_manipController, 90, 0).OnTrue(new frc2::InstantCommand([this] { 
        m_leds.SetState(0); 
        Logger::Log(LogLevel::All) << " Hypno LEDS ENABLED" << LoggerCommand::Flush;
    }, {&m_leds}));
    frc2::POVButton(&m_manipController, 270, 0).OnTrue(new frc2::InstantCommand([this] { 
        m_leds.SetState(0); 
        Logger::Log(LogLevel::All) << " Hypno LEDS ENABLED" << LoggerCommand::Flush;
    }, {&m_leds}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    auto selected_command = AutoConstants::kAutoSequences[AutoConstants::kSelectedAuto].Commands[m_auto_command_index];
    m_auto_command_index++;

    Logger::Log(LogLevel::Dev) << "Getting Command: " << std::to_string(selected_command.CommandType) << LoggerCommand::Flush;

    // TODO: Update these
    switch(selected_command.CommandType) {
        case 0:
            return new AutoMoveTimedCommand(m_wrapper, selected_command);
        case 1:
            return new AutoMoveDistanceCommand(m_wrapper, selected_command);
        case 20:
            return new AutoArmMoveCommand(m_wrapper, selected_command);
        case 22:
            return new AutoDriveClawCommand(m_wrapper, selected_command);
        case 30:
            return new AutoTimedWaitCommand(m_wrapper, selected_command);
        default:
            return new frc2::InstantCommand([this, selected_command]() { Logger::Log(LogLevel::Autonomous) << "Command: [" << std::to_string(selected_command.CommandType) << "] not implemented!!!" << LoggerCommand::Flush; }, {});
    }
}

void RobotContainer::InitTeleop() {
    m_arm.SetCollectUseState(false);
    m_drive.SetSpeed(DriveConstants::kDefaultSlow ? DriveConstants::kLowSpeed : DriveConstants::kFastSpeed);
}

void RobotContainer::ResetArmState() {
    m_arm.SetState(-1);
    m_arm.SetCollectUseState(false);
}

void RobotContainer::ResetAutoCommandCount() {
    m_auto_command_index = 0;
}

void RobotContainer::PostConfigInit() {
    //m_arm.SetClaw(0); // TODO: Implement this
}