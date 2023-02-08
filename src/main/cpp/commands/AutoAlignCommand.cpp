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
    double x = m_limelight.GetX()+AutoConstants::kAutoTargetX;
    double z = m_limelight.GetZ()+AutoConstants::kAutoTargetZ;
    double angle = m_limelight.GetHeading();
    
    Logger::log(LogLevel::Info) << "AutoAlignCommand x: " << x << " z: " << z << " angle: " << angle << "\n";

    x = std::clamp(5*x, -1.0, 1.0);
    z = std::clamp(-5*z, -1.0, 1.0);
    angle = std::clamp(-angle/30.0, -1.0, 1.0);

    Logger::log(LogLevel::Info) << "After clamping   x: " << x << " z: " << z << " angle: " << angle << "\n";

    units::meters_per_second_t x_u{x};
    units::meters_per_second_t z_u{z};
    units::radians_per_second_t angle_u{angle * std::numbers::pi};

    m_drive.Drive(x_u, z_u, angle_u, false, true); // TODO: 
}

// Called once the command ends or is interrupted.
void AutoAlignCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoAlignCommand::IsFinished() {return false;}
