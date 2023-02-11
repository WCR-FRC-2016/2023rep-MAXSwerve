/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Limelight.h"
#include "Constants.h"

Limelight::Limelight() {
	//wpi::outs() << "Limelight constructed\n"
    
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

void Limelight::Periodic() {}

// Returns true if a target is detected.
bool Limelight::GetVisible() {
    return table->GetBoolean("tv", false);
}

// Returns x position of target in robot space.
double Limelight::GetX() {
    return table->GetNumberArray("targetpose_robotspace", zeroes)[0];
}

// Returns z position of target in robot space.
double Limelight::GetZ() {
    return table->GetNumberArray("targetpose_robotspace", zeroes)[2];
}

// Returns heading of robot in target space.
double Limelight::GetHeading() {
    return -table->GetNumberArray("targetpose_robotspace", zeroes)[4];
}