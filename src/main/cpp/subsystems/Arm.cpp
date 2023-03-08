// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"
#include "utils/SwerveUtils.h"

#include "Logging.hpp"

using namespace ArmConstants;

Arm::Arm()
    : m_hand_left(kHandLeftId, rev::CANSparkMax::MotorType::kBrushed),
      m_hand_right(kHandRightId, rev::CANSparkMax::MotorType::kBrushed),
      m_hand_grab(kHandGrabId, rev::CANSparkMax::MotorType::kBrushed),
      m_low_actuator(1), // TODO: ids
      m_high_actuator(2), // TODO: ids
      m_arm_low_pid{kArmLowP, kArmLowI, kArmLowD},
      m_arm_high_pid{kArmHighP, kArmHighI, kArmHighD},
      m_low_encoder{0},
      m_high_encoder{1} {
  // Factory reset, so we get the SPARKS MAX to a known state before configuring
  // them. This is useful in case a SPARK MAX is swapped out.
  m_hand_left.RestoreFactoryDefaults();
  m_hand_right.RestoreFactoryDefaults();
  m_hand_grab.RestoreFactoryDefaults();
  
  m_hand_left.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_hand_right.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_hand_grab.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_hand_left.SetSmartCurrentLimit(kHandLeftCurrentLimit.value());
  m_hand_right.SetSmartCurrentLimit(kHandRightCurrentLimit.value());
  m_hand_grab.SetSmartCurrentLimit(kHandGrabCurrentLimit.value());
}

void Arm::Periodic() {
  if (m_state==-1) return;

  double low, high;
  switch (m_goal_state) {
    case 1: // Carry
      low = 0; high = 0;
      break;
    case 2: // Medium Tier
    case 3: // High Tier
    case 4: // Substation
    case 5: // Intermediate
    case -1:
    default:
      return;
  }

  TurnToAngles(low, high);

  if (m_arm_low_pid.AtSetpoint() && m_arm_high_pid.AtSetpoint()) {
    m_state = m_goal_state;
    m_goal_state = m_next_goal_state;
    m_next_goal_state = -1;
  }
}

void Arm::SetState(double new_state) {
  if ((m_state==1 && new_state!=1) || (m_state!=1 && new_state==1)) {
    m_goal_state = 5;
    m_next_goal_state = new_state;
  } else {
    m_goal_state = new_state;
    m_next_goal_state = -1;
  }
}

void Arm::TurnToAngles(double low, double high) {
  double low_move = m_arm_low_pid.Calculate(m_low_encoder.GetDistance(), low);
  double high_move = m_arm_high_pid.Calculate(m_high_encoder.GetDistance(), high);

  low_move = std::clamp(low_move, -1.0, 1.0);
  high_move = std::clamp(high_move, -1.0, 1.0);

  m_low_actuator.Drive(low_move);
  m_high_actuator.Drive(high_move);
}

void Arm::Drive(double low, double high) {
  m_state = -1;
  m_low_actuator.Drive(low);
  m_high_actuator.Drive(high);
}

void Arm::PrintTestEncoder() {
  frc::SmartDashboard::PutBoolean("LowEncoderConnected", m_low_encoder.IsConnected());
  frc::SmartDashboard::PutNumber("LowEncoderDistance", m_low_encoder.GetDistance());
  frc::SmartDashboard::PutNumber("LowEncoderDistanceDegrees", m_low_encoder.GetDistanceDegrees().value());
  frc::SmartDashboard::PutNumber("LowEncoderNormalizedDistanceDegrees", m_low_encoder.GetNormalizedDistanceDegrees().value());
  frc::SmartDashboard::PutNumber("LowEncoderRevolutions", m_low_encoder.GetRevolutions());

  frc::SmartDashboard::PutBoolean("HighEncoderConnected", m_high_encoder.IsConnected());
  frc::SmartDashboard::PutNumber("HighEncoderDistance", m_high_encoder.GetDistance());
  frc::SmartDashboard::PutNumber("HighEncoderDistanceDegrees", m_high_encoder.GetDistanceDegrees().value());
  frc::SmartDashboard::PutNumber("HighEncoderNormalizedDistanceDegrees", m_high_encoder.GetNormalizedDistanceDegrees().value());
  frc::SmartDashboard::PutNumber("HighEncoderRevolutions", m_high_encoder.GetRevolutions());
}