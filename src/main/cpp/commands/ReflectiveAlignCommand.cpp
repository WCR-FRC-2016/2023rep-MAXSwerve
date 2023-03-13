/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ReflectiveAlignCommand.h"
#include "Constants.h"

ReflectiveAlignCommand::ReflectiveAlignCommand(DriveSubsystem& drive, Limelight& limelight) : m_drive(drive), m_limelight(limelight) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({&drive, &limelight});
}

// Called when the command is initially scheduled.
void ReflectiveAlignCommand::Initialize() {
  m_limelight.Activate();
  m_limelight.SetPipeline(1);
}

// Called repeatedly when this Command is scheduled to run
void ReflectiveAlignCommand::Execute() {
  if (m_limelight.GetVisible()) {
    double x = (m_limelight.GetReflectiveX() - AutoConstants::kAutoReflectiveTargetX.value()) / 10.0;
    
    Logger::Log(LogLevel::Dev) << "ReflectiveAlignCommand x: " << x << LoggerCommand::Flush;

    if (abs(x) < AutoConstants::kAutoTargetDeadzone.value()) x = 0;

    units::meters_per_second_t x_u = units::meters_per_second_t{std::clamp(-AutoConstants::kAlignSpeed*x, -1.0, 1.0)};
    //units::meters_per_second_t z_u = units::meters_per_second_t{std::clamp(AutoConstants::kAlignSpeed*z, -1.0, 1.0)};
    //units::radians_per_second_t angle_u = units::radians_per_second_t{std::clamp(AutoConstants::kAlignRotationSpeed*angle/360.0, -1.0, 1.0)};

    Logger::Log(LogLevel::Dev) << "After clamping x: " << x_u << LoggerCommand::Flush;

    m_drive.Drive(0_mps, x_u, 0_rad_per_s, false, true);
  } else {
    Logger::Log(LogLevel::Dev | LogLevel::Important) << "Target not found!" << LoggerCommand::Flush;
    
    m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
  }
}

// Called once the command ends or is interrupted.
void ReflectiveAlignCommand::End(bool interrupted) {
  m_limelight.Deactivate();
}

// Returns true when the command should end.
bool ReflectiveAlignCommand::IsFinished() {return false;}

template <typename T>
T ReflectiveAlignCommand::Abs(T x) {
  return x.value()<0 ? -x : x;
}