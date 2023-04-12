/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/MoveClawCommand.h"
#include "Constants.h"

// 1 opens, -1 closes. Fractional speeds should be supported (TODO: test)
MoveClawCommand::MoveClawCommand(Arm& arm, double dir) : m_arm(arm), m_dir(dir) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({&arm});
}

// Called when the command is initially scheduled.
void MoveClawCommand::Initialize() {
    m_frames = 0;
}

// Called repeatedly when this Command is scheduled to run
void MoveClawCommand::Execute() {
    Logger::Log(LogLevel::Dev) << "Finished?: " << IsFinished() << "\n";
    Logger::Log(LogLevel::Dev) << "Direction: " << m_dir << "\n";
    Logger::Log(LogLevel::Dev) << "Position: " << m_arm.GetClawPos() << LoggerCommand::Flush;

    bool at_end;
    if (m_dir>0) at_end = m_arm.GetOuterLimitSwitchState();
    else at_end = m_arm.GetInnerLimitSwitchState();

    if (at_end) {
        m_arm.DriveClaw(0);
        m_frames++;
    } else {
        m_arm.DriveClaw(m_dir);
        m_frames=0;
    }
}

// Called once the command ends or is interrupted.
void MoveClawCommand::End(bool interrupted) {
    m_arm.DriveClaw(0);
    
}

// Returns true when the command should end.
bool MoveClawCommand::IsFinished() {
    if (ArmConstants::kUseTiming) {
        if (m_dir>0) return m_arm.GetClawPos()<=0;
        else return m_arm.GetClawPos()>=ArmConstants::kClawMoveTime;
    } else {
        return m_frames >= 3;
    }
}