/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoAlignCommand.h"
#include "Constants.h"

AutoAlignCommand::AutoAlignCommand(DriveSubsystem& drive, Limelight& limelight) : m_drive(drive), m_limelight(limelight) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({&drive, &limelight});
}

// Called when the command is initially scheduled.
void AutoAlignCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoAlignCommand::Execute() {
  if (m_limelight.GetVisible()) {
    double x = m_limelight.GetX() - AutoConstants::kAutoTargetX.value();
    double z = m_limelight.GetZ() - AutoConstants::kAutoTargetZ.value();
    double angle = m_limelight.GetHeading();
    
    Logger::Log(LogLevel::Dev) << "AutoAlignCommand x: " << x << " z: " << z << " angle: " << angle << LoggerCommand::Flush;

    if (abs(x) < AutoConstants::kAutoTargetDeadzone.value()) x = 0;
    if (abs(z) < AutoConstants::kAutoTargetDeadzone.value()) z = 0;
    if (abs(angle) < units::degree_t{AutoConstants::kAutoTargetAngularDeadzone}.value()) angle = 0;

    units::meters_per_second_t x_u = units::meters_per_second_t{std::clamp(-DriveConstants::kAlignSpeed*x, -1.0, 1.0)};
    units::meters_per_second_t z_u = units::meters_per_second_t{std::clamp(DriveConstants::kAlignSpeed*z, -1.0, 1.0)};
    units::radians_per_second_t angle_u = units::radians_per_second_t{std::clamp(angle/360.0, -1.0, 1.0)};

    Logger::Log(LogLevel::Dev) << "After clamping x: " << x_u << " z: " << z_u << " angle: " << angle_u << LoggerCommand::Flush;

    m_drive.Drive(z_u, x_u, angle_u, false, true);
  } else {
    Logger::Log(LogLevel::Dev | LogLevel::Important) << "Target not found!" << LoggerCommand::Flush;
    
    m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
  }
}

// Called once the command ends or is interrupted.
void AutoAlignCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoAlignCommand::IsFinished() {return false;}

template <typename T>
T AutoAlignCommand::Abs(T x) {
  return x.value()<0 ? -x : x;
}