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
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "Logging.hpp"
#include "commands/AutoAlignCommand.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
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
        if (m_arm.GetGoalState()==-2) {
            m_arm.Drive(
                frc::ApplyDeadband(m_manipController.GetRightY(), IOConstants::kDriveDeadband),
                frc::ApplyDeadband(-m_manipController.GetLeftY(), IOConstants::kDriveDeadband)
            );
            //Logger::Log(LogLevel::Dev) << "Arm raw lower angle: " << m_arm.GetRawLowerAngle() << LoggerCommand::Flush;
            Logger::Log(LogLevel::Dev) << "Arm lower angle: " << m_arm.GetLowerAngle() << "\n";
            Logger::Log(LogLevel::Dev) << "Arm upper angle: " << m_arm.GetUpperAngle() << "\n" << LoggerCommand::Flush;
        }
      },
      {&m_arm}));
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController, ControlConstants::xModeButton)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

  // Temporary Commands

  // Toggle Field Relative
  frc2::JoystickButton(&m_driverController, ControlConstants::RelativeButton)
      .OnTrue(
          new frc2::InstantCommand([this] { m_relative ^= true; }, {&m_drive}));

  // Toggle RateLimit
  frc2::JoystickButton(&m_driverController, ControlConstants::RateLimitButton)
      .OnTrue(new frc2::InstantCommand([this] { m_rate_limit ^= true; },
                                       {&m_drive}));

  // Log some information
  frc2::JoystickButton(&m_driverController, ControlConstants::DebugPrintButton)
      .OnTrue(new frc2::InstantCommand(
          [this] {
            Logger::Log(LogLevel::Dev)
                << "Speed [" << m_drive.GetSpeed().value()
                << " mps], Relative: [" << m_relative << "], RateLimit: ["
                << m_rate_limit << "], Gyro Angle: ["
                << m_drive.GetHeading().value() << "]" << LoggerCommand::Flush;
          },
          {&m_drive}));

  frc2::JoystickButton(&m_driverController, ControlConstants::AlignATButton)
      .WhileTrue(new AutoAlignCommand(m_drive, m_limelight, 0));

  frc2::JoystickButton(&m_driverController, ControlConstants::AlignRTButton)
      .WhileTrue(new AutoAlignCommand(m_drive, m_limelight, 1));

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

  frc2::JoystickButton(&m_driverController, ControlConstants::PosButton)
      .OnTrue(new frc2::InstantCommand(
          [this] {
            Logger::Log(LogLevel::Dev)
                << "Position: " << m_drive.GetPose().Translation()
                << LoggerCommand::Flush;
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
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

    frc::Trajectory trajectory;
    trajectory = frc::TrajectoryGenerator::GenerateTrajectory({
                        frc::Pose2d{0_m, 0_m, 0_deg},
                        frc::Pose2d(1.0_m, 0.0_m, 90_deg)
                    },
                    config);
    // switch(AutoConstants::kSelectedAuto) {
    //     case 1:
    //         trajectory = frc::TrajectoryGenerator::GenerateTrajectory({
    //                     frc::Pose2d{1_m, 0_m, 0_deg},
    //                     frc::Pose2d(0.0_m, 0.0_m, 90_deg)
    //                 },
    //                 config);
    //         break;
    //     case 2:
    //         trajectory = frc::TrajectoryGenerator::GenerateTrajectory({
    //                     frc::Pose2d{1_m, 0_m, 0_deg},
    //                     frc::Pose2d(0.0_m, 0.0_m, 0_deg)
    //                 },
    //                 config);
    //         break;
    //     case 0:
    //     default:
    //         trajectory = frc::TrajectoryGenerator::GenerateTrajectory({
    //                     frc::Pose2d{0_m, 0_m, 0_deg},
    //                     frc::Pose2d(0.0_m, 0.0_m, 90_deg)
    //                 },
    //                 config);
    //         break;
    // }

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      trajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController{AutoConstants::kPXController, 0, 0},
      frc2::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(trajectory.InitialPose());

    return new frc2::SequentialCommandGroup(
        std::move(swerveControllerCommand),
        frc2::InstantCommand([this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); })
        );

  // no auto
  //return new frc2::RunCommand([this]() { m_arm.PrintTestEncoder(); });

  //   return new frc2::SequentialCommandGroup(
  //     frc2::InstantCommand([this]() {
  //         Logger::SetGlobalLevel(LogLevel::Dev);
  //        Logger::Log(LogLevel::Dev) << "Position: " <<
  //        m_drive.GetPose().Translation() << LoggerCommand::Flush;},
  //        {&m_drive}),
  //       std::move(swerveControllerCommand),
  //       frc2::InstantCommand(
  //           [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false,
  //           false); },
  //           {}),
  //               frc2::InstantCommand([this]() {
  //         Logger::SetGlobalLevel(LogLevel::Dev);
  //        Logger::Log(LogLevel::Dev) << "Position: " <<
  //        m_drive.GetPose().Translation() << LoggerCommand::Flush;},
  //        {&m_drive}));
}

void RobotContainer::ResetArmState() {
    m_arm.SetState(-1);
}