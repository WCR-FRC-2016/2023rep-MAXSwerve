/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc2/command/SubsystemBase.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

class Limelight : public frc2::SubsystemBase {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  std::shared_ptr<nt::NetworkTable> table;

  double zeroes[6]{0,0,0,0,0,0}; // Used as default for calls to GetNumberArray.

 public:
  Limelight();
  void Periodic();
  void SetPipeline(int pipeline);
  bool GetVisible();
  double GetTargetPoseX();
  double GetReflectiveX();
  double GetTargetPoseZ();
  double GetHeading();
};
