// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>

#include "CTREMagEncoder.h"
#include "Constants.h"

class Arm : public frc2::SubsystemBase {
 public:
  Arm();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.
  void TurnToAngles(double low, double high);
  void PrintTestEncoder();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkMax m_hand_left; // Motor to control left hand wheels
  rev::CANSparkMax m_hand_right; // Motor to control right hand wheels
  rev::CANSparkMax m_hand_grab; // Motor to control hand opening/closing
  rev::CANSparkMax m_arm_low; // Motor to control lower arm
  rev::CANSparkMax m_arm_high; // Motor to control upper arm
  
  frc2::PIDController m_arm_low_pid;
  frc2::PIDController m_arm_high_pid;

  CTREMagEncoder m_low_encoder;
  CTREMagEncoder m_high_encoder;
};
