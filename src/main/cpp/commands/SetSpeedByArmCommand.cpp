/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetSpeedByArmCommand.h"
#include "Constants.h"

SetSpeedByArmCommand::SetSpeedByArmCommand(DriveSubsystem& drive, Arm& arm, std::function<double()> brake, std::function<double()> turbo) : m_drive(drive), m_arm(arm), m_brake(brake), m_turbo(turbo) {
}

// Called when the command is initially scheduled.
void SetSpeedByArmCommand::Initialize() { }

// Called repeatedly when this Command is scheduled to run
void SetSpeedByArmCommand::Execute() {
    // double anglePercentage = m_arm.GetUpperAngle() / 80_deg;
    units::degree_t angle = m_arm.GetUpperAngle();
    if (angle > 270_deg) angle -= 360_deg;
    double speedFactor = std::clamp(1.0 - 0.1*(angle/80_deg).value(), 0.9, 1.0); // Arm Lift Brake (1x - 0.9x)
    //Logger::Log(LogLevel::Match) << "Speed factor with arm: " << speedFactor << LoggerCommand::Flush;

    speedFactor *= (1.0 - m_brake() * 0.72);  // Left Trigger Brake (1x - 0.28x)
    speedFactor *= (1.0 + m_turbo() * 0.4); // Right Trigger Turbo (1x - 1.4x)

    Logger::Log(LogLevel::Dev) << "Speed factor with turbo/brake: " << speedFactor << LoggerCommand::Flush;

    m_drive.SetSpeedFactor(speedFactor);
}

// Called once the command ends or is interrupted.
void SetSpeedByArmCommand::End(bool interrupted) { }

// Returns true when the command should end.
bool SetSpeedByArmCommand::IsFinished() {return false;}