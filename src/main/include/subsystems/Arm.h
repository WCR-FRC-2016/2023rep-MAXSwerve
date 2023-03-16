// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>

#include <units/angle.h>

#include "ActuatorModule.h"
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
  void SetState(double new_state);
  double GetGoalState();
  void TurnToAngles(units::degree_t low, units::degree_t high);
  void Drive(double low, double high);
  void PrintTestEncoder();

  units::degree_t GetUpperAngle();
  units::degree_t GetLowerAngle();
  double GetRawUpperAngle();
  double GetRawLowerAngle();

  bool GetOuterLimitSwitchState();
  bool GetInnerLimitSwitchState();

  int GetClawPos();
  void SetCollectUseState(bool state);
  void SetCollectState(int32_t state);
  //void SetClawUseState(bool state);
  //void SetClawState(int32_t state);

  void DriveClaw(double dir);
  void DriveCollectWheels(double dir);

  void SetClaw(int32_t claw_settings);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkMax m_hand_left; // Motor to control left hand wheels
  rev::CANSparkMax m_hand_right; // Motor to control right hand wheels
  rev::CANSparkMax m_hand_grab; // Motor to control hand opening/closing
  
  ActuatorModule m_low_actuator;
  ActuatorModule m_high_actuator;
  
  frc2::PIDController m_arm_low_pid;
  frc2::PIDController m_arm_high_pid;

  CTREMagEncoder m_low_encoder;
  CTREMagEncoder m_high_encoder;

  frc::DigitalInput m_outer_switch;
  frc::DigitalInput m_inner_switch;

  int m_state = 1;
  int m_goal_state = -1;
  int m_next_goal_state = -1;

  bool m_use_collect_state = false;
  int m_collect_state = 0;

  //bool m_use_claw_state = false;
  // Cube = 0, Cone = 1

  // Warning: direction inverted from DriveClaw.
  int m_current_pos = 0;
};
