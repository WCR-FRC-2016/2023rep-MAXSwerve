// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
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
#include "subsystems/DriveSubsystem.h"
#include "commands/AutoAlignCommand.h"

#include "Logging.hpp"

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

        float_t dir = static_cast<float_t>(m_manipController.GetLeftY());
        //frc::SmartDashboard::PutNumber("ManipLeft", dir);
        m_drive.m_actuator.Drive(-dir);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController,
                       ControlConstants::xModeButton)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

    // Temporary Commands

    // Toggle Field Relative
    frc2::JoystickButton(&m_driverController, 
    ControlConstants::RelativeButton).OnTrue(new frc2::InstantCommand([this] { m_relative ^= true; }, {&m_drive}));

    // Toggle RateLimit
    frc2::JoystickButton(&m_driverController, 
    ControlConstants::RateLimitButton).OnTrue(new frc2::InstantCommand([this] { m_rate_limit ^= true; }, {&m_drive}));

    //Log some information
    frc2::JoystickButton(&m_driverController,
    ControlConstants::DebugPrintButton).OnTrue(new frc2::InstantCommand([this] {
       Logger::Log(LogLevel::Dev) << "Speed [" << m_drive.GetSpeed().value() << " mps], Relative: [" << m_relative << "], RateLimit: [" << m_rate_limit  << "], Gyro Angle: [" << m_drive.GetHeading().value() << "]" << LoggerCommand::Flush;
    }, {&m_drive}));

    frc2::JoystickButton(&m_driverController, 
    ControlConstants::AlignButton).WhileTrue(new AutoAlignCommand(m_drive, m_limelight));

    frc2::JoystickButton(&m_driverController, 
    ControlConstants::SwapSpeedButton).OnTrue(new frc2::InstantCommand([this] {
         if (m_low_speed) m_drive.SetSpeed(DriveConstants::FastSpeed);
         else m_drive.SetSpeed(DriveConstants::LowSpeed);
        m_low_speed ^= true;
    }, {&m_drive}));

    frc2::JoystickButton(&m_driverController, 
    ControlConstants::PosButton).OnTrue(new frc2::InstantCommand([this] {
       Logger::Log(LogLevel::Dev) << "Position: " << m_drive.GetPose().Translation() << LoggerCommand::Flush;
    }, {&m_drive}));

    // Map this to potentially reset just the rotation and not the position???
    frc2::JoystickButton(&m_driverController,
    ControlConstants::ResetHeadingButton).OnTrue(new frc2::InstantCommand([this] {
        m_drive.ZeroHeading();
    }, {&m_drive}));

    // LED State Change Command
    frc2::JoystickButton(&m_driverController,
    ControlConstants::DebugLEDButton).OnTrue(new frc2::InstantCommand([this] {
        m_leds.SetState((m_leds.GetState()+1)%3);
    }, {&m_leds}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1.0_m, 0.0_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{3_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController{AutoConstants::kPXController, 0, 0},
      frc2::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
    return new frc2::RunCommand([this]() {
        m_arm.PrintTestEncoder();
    });

//   return new frc2::SequentialCommandGroup(
//     frc2::InstantCommand([this]() { 
//         Logger::SetGlobalLevel(LogLevel::Dev);
//        Logger::Log(LogLevel::Dev) << "Position: " << m_drive.GetPose().Translation() << LoggerCommand::Flush;}, {&m_drive}),
//       std::move(swerveControllerCommand),
//       frc2::InstantCommand(
//           [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); },
//           {}),
//               frc2::InstantCommand([this]() { 
//         Logger::SetGlobalLevel(LogLevel::Dev);
//        Logger::Log(LogLevel::Dev) << "Position: " << m_drive.GetPose().Translation() << LoggerCommand::Flush;}, {&m_drive}));
}
