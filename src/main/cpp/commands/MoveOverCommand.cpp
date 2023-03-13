/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/MoveOverCommand.h"
#include "Constants.h"

MoveOverCommand::MoveOverCommand(DriveSubsystem& drive, double time) : m_drive(drive), m_time(time) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({&drive});
}

// Called when the command is initially scheduled.
void MoveOverCommand::Initialize() {
    m_elapsed = 0;
}

// Called repeatedly when this Command is scheduled to run
void MoveOverCommand::Execute() {
    // Note: -y is to the right (obviously)
    m_drive.Drive(0_mps, m_time>0?-1_mps:1_mps, 0_rad_per_s, false, false);

    m_elapsed += 20;
}

// Called once the command ends or is interrupted.
void MoveOverCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool MoveOverCommand::IsFinished() {return m_elapsed>=abs(m_time);}