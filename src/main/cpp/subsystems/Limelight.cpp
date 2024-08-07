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

void Limelight::Activate() {
    table->PutNumber("camMode", 0);
    table->PutNumber("ledMode", 0);
}

void Limelight::Deactivate() {
    table->PutNumber("camMode", 1);
    table->PutNumber("ledMode", 1);
}

void Limelight::SetPipeline(int pipeline) {
    table->PutNumber("pipeline", pipeline);
}

// Returns true if a target is detected.
bool Limelight::GetVisible() {
    return table->GetNumber("tv", 0.0)>0.5;
}

// Returns x position of target in robot space.
double Limelight::GetTargetPoseX() {
    return table->GetNumberArray("targetpose_robotspace", zeroes)[0];
}

// Returns z position of target in robot space.
double Limelight::GetTargetPoseZ() {
    return table->GetNumberArray("targetpose_robotspace", zeroes)[2];
}

// Returns x rotation of reflective tape in robot space
double Limelight::GetReflectiveX() {
    return table->GetNumber("tx", 0.0);
}

// Returns heading of robot in target space.
double Limelight::GetHeading() {
    return -table->GetNumberArray("targetpose_robotspace", zeroes)[4];
}

// Returns ID of target.
int Limelight::GetTargetID() {
    return table->GetNumber("tid", -1);
}

// Returns true if id detected is 4 or 5, false otherwise.
bool Limelight::IsSubstation() {
    return GetTargetID()==4 || GetTargetID()==5;
}