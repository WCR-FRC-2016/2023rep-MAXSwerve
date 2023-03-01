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
      m_arm_low(kArmLowId, rev::CANSparkMax::MotorType::kBrushed),
      m_arm_high(kArmHighId, rev::CANSparkMax::MotorType::kBrushed),
      m_low_encoder{0},
      m_high_encoder{1} {
  // Factory reset, so we get the SPARKS MAX to a known state before configuring
  // them. This is useful in case a SPARK MAX is swapped out.
  m_hand_left.RestoreFactoryDefaults();
  m_hand_right.RestoreFactoryDefaults();
  m_hand_grab.RestoreFactoryDefaults();
  m_arm_low.RestoreFactoryDefaults();
  m_arm_high.RestoreFactoryDefaults();
  
  m_hand_left.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_hand_right.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_hand_grab.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_arm_low.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_arm_high.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_hand_left.SetSmartCurrentLimit(kHandLeftCurrentLimit.value());
  m_hand_right.SetSmartCurrentLimit(kHandRightCurrentLimit.value());
  m_hand_grab.SetSmartCurrentLimit(kHandGrabCurrentLimit.value());
  m_arm_low.SetSmartCurrentLimit(kArmLowCurrentLimit.value());
  m_arm_high.SetSmartCurrentLimit(kArmHighCurrentLimit.value());

  m_arm_low_pid.SetP(kArmLowP);
  m_arm_low_pid.SetI(kArmLowI);
  m_arm_low_pid.SetD(kArmLowD);

  m_arm_high_pid.SetP(kArmHighP);
  m_arm_high_pid.SetI(kArmHighI);
  m_arm_high_pid.SetD(kArmHighD);
}

void Arm::Periodic() {
    // TODO: Use PID loops to control motors
}

void Arm::TurnToAngles(double low, double high) {
    m_arm_low_pid.SetSetpoint(low);
    m_arm_high_pid.SetSetpoint(high);
}

void Arm::PrintTestEncoder() {
  frc::SmartDashboard::PutBoolean("LowEncoderConnected", m_low_encoder.IsConnected());
  frc::SmartDashboard::PutNumber("LowEncoderDistance", m_low_encoder.GetDistance());
  frc::SmartDashboard::PutNumber("LowEncoderDistanceDegrees", m_low_encoder.GetDistanceDegrees().value());
  frc::SmartDashboard::PutNumber("LowEncoderDistanceDegreesNormalized", m_low_encoder.GetNormalizedDistanceDegrees().value());
  frc::SmartDashboard::PutNumber("LowEncoderRevolutions", m_low_encoder.GetRevolutions());

  frc::SmartDashboard::PutBoolean("HighEncoderConnected", m_high_encoder.IsConnected());
  frc::SmartDashboard::PutNumber("HighEncoderDistance", m_high_encoder.GetDistance());
  frc::SmartDashboard::PutNumber("HighEncoderDistanceDegrees", m_high_encoder.GetDistanceDegrees().value());
  frc::SmartDashboard::PutNumber("HighEncoderDistanceDegreesNormalized", m_high_encoder.GetNormalizedDistanceDegrees().value());
  frc::SmartDashboard::PutNumber("HighEncoderRevolutions", m_high_encoder.GetRevolutions());
}