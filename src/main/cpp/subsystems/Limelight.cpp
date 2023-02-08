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

void Limelight::LimelightInit() {
	//wpi::outs() << "Limelight initialized\n";
    initialized = true;
}

void Limelight::Periodic() {
	if (!initialized) {
		Limelight::LimelightInit();
	}
}

double Limelight::GetX() {
    return table->GetNumberArray("campose", zeroes)[0];
}

double Limelight::GetZ() {
    return table->GetNumberArray("campose", zeroes)[2];
}

double Limelight::GetHeading() {
    return table->GetNumberArray("campose", zeroes)[4];
}