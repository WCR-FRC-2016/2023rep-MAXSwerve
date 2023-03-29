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

// Top Arm at 90 degrees: 26.6 degrees
// Bottom arm at 0: 49.2 degrees

// ABSOLUTE 0: (both limit switches in actuators are hit)
// Bottom: 47.1
// Upper: -52.5

// Grabbing Low:
// Bottom: 50
// Upper: -44.1

Arm::Arm()
    : m_hand_left(kHandLeftId, rev::CANSparkMax::MotorType::kBrushed),
      m_hand_right(kHandRightId, rev::CANSparkMax::MotorType::kBrushed),
      m_hand_grab(kHandGrabId, rev::CANSparkMax::MotorType::kBrushed),
      m_low_actuator(kArmLowId),
      m_high_actuator(kArmHighId),
      m_arm_low_pid{kArmLowP, kArmLowI, kArmLowD},
      m_arm_high_pid{kArmHighP, kArmHighI, kArmHighD},
      m_low_encoder{kArmLowEncoderId},
      m_high_encoder{kArmHighEncoderId},
      m_outer_switch{0},
      m_inner_switch{1} {
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

  m_hand_left.BurnFlash();
  m_hand_right.BurnFlash();
  m_hand_grab.BurnFlash();

  m_arm_low_pid.EnableContinuousInput(0.0, 360.0);
  m_arm_high_pid.EnableContinuousInput(0.0, 360.0);

  m_arm_low_pid.SetTolerance(1.5);
  m_arm_high_pid.SetTolerance(1.5);
}

void Arm::Periodic() {
  units::degree_t low, high;
  switch (m_goal_state) {
    case 1: // Floor
      low = -22.2_deg; high = 0_deg;
      break;
    case 2: // Medium Tier
      low = -23_deg; high = 64.4_deg;
      break;
    case 3: // High Tier
      low = -43.5_deg; high = 108.0_deg;
      break;
    case 4: // Substation
      low = -23_deg; high = 71.4_deg;
      break;
    case 5: // Carry / Intermediate
    case 6: // Absolute Zero
      low = 0_deg; high = 0_deg;
      break;
    case 7: // Right Angle
      low = -23.0_deg; high = 79.1_deg;
      break;
    case -1:
    case -2:
    default:
      return;
  }

  TurnToAngles(low, high);
  if (m_arm_low_pid.AtSetpoint() && m_arm_high_pid.AtSetpoint()) {
    m_state = m_goal_state;
    m_goal_state = m_next_goal_state;
    m_next_goal_state = -1;
    Logger::Log(LogLevel::Autonomous) << "Finished" << LoggerCommand::Flush;
  }

  // Only on Claw Design 2
  // TODO: m_hand_grab.GetOutputCurrent()
}

void Arm::SetState(double new_state) {
  /*if ((m_state==1 && new_state!=1) || (m_state!=1 && new_state==1)) {
    m_goal_state = 5;
    m_next_goal_state = new_state;
  } else {
    m_goal_state = new_state;
    m_next_goal_state = -1;
  }*/
  m_goal_state = new_state;
  m_next_goal_state = -1;
}

double Arm::GetGoalState() {
  return m_goal_state;
}

void Arm::TurnToAngles(units::degree_t low, units::degree_t high) {
  double low_move = m_arm_low_pid.Calculate(GetLowerAngle().value(), low.value());
  double high_move = m_arm_high_pid.Calculate(GetUpperAngle().value(), high.value());

  Logger::Log(LogLevel::Dev) << "low: " << low                              << " " << "high: " << high << "\n";
  Logger::Log(LogLevel::Dev) << "lowcurrent: " << GetLowerAngle()           << " " << "highcurrent: " << GetUpperAngle() << "\n";
  Logger::Log(LogLevel::Dev) << "low_move: " << low_move                    << " " << "high_move: " << high_move << "\n";
  Logger::Log(LogLevel::Dev) << "low good?: " << m_arm_low_pid.AtSetpoint() << " " << "high good?: " << m_arm_high_pid.AtSetpoint() << LoggerCommand::Flush;

  low_move = std::clamp(low_move, -1.0, 1.0);
  high_move = std::clamp(high_move, -1.0, 1.0);

  if (!m_arm_low_pid.AtSetpoint())   m_low_actuator.Drive(low_move);
  if (!m_arm_high_pid.AtSetpoint()) m_high_actuator.Drive(high_move);

  //if (m_use_claw_state) DriveClaw(m_claw_state);
}

bool Arm::GetOuterLimitSwitchState() { return m_outer_switch.Get(); }
bool Arm::GetInnerLimitSwitchState() { return m_inner_switch.Get(); }

//void Arm::SetClawState(int32_t state)    { m_claw_state = state; }
//void Arm::SetClawUseState(bool state)    { m_use_claw_state = state; }
void Arm::SetCollectUseState(bool state) { m_use_collect_state = state; }
void Arm::SetCollectState(int32_t state) { m_collect_state = state; }

// -1 drives closed
//  1 drives open
void Arm::DriveClaw(double dir) {
  //Logger::Log(LogLevel::All) << "Outer: " << m_outer_switch.Get() << ", Inner: " << m_inner_switch.Get() << LoggerCommand::Flush;

  // TODO: Fix Limit Switches (still dont work)
  if (ArmConstants::kUseLimitSwitches) {
    if (m_outer_switch.Get()) {
      Logger::Log(LogLevel::Dev) << "Outer Switch Active!" << LoggerCommand::Flush;
      //m_current_pos = 0;
      if (dir>0) dir = 0;

      //m_claw_state = 0;
      //m_use_claw_state = false;
    }

    if (m_inner_switch.Get()) {
      Logger::Log(LogLevel::Dev) << "Inner Switch Active!" << LoggerCommand::Flush;
      //m_current_pos = ArmConstants::kClawMoveTime;
      if (dir<0) dir = 0;

      //m_claw_state = 0;
      //m_use_claw_state = false;
    }
  }

  /*
  // Verify we aren't using claw state when it's 0
  if (m_use_claw_state && m_claw_state == 0) 
    m_use_claw_state = false;
  */
  
  if (ArmConstants::kUseTiming) {
    if (dir<0 && m_current_pos>=ArmConstants::kClawMoveTime) return;
    if (dir>0 && m_current_pos<=0) return;

    m_current_pos -= dir*20;
  }

  m_hand_grab.Set(dir);
}

int Arm::GetClawPos() { return m_current_pos; }

// WARNING: Don't use this unless you know what you're doing (may break the claw otherwise)
// 0 is fully cube position
// ArmConstants::kClawMoveTime is fully cone position
void Arm::OverrideClawPos(double new_pos) { m_current_pos = new_pos; }

bool Arm::HasPiece() { return !m_hasPieceSensor.Get(); }

//  1 Sucks in
// -1 Spits out
void Arm::DriveCollectWheels(double dir) {
  // If collecting and piece has already been collected, don't run motors
  if (dir > 0 && HasPiece()) {
    m_hand_right.Set(0.0);
    m_hand_left.Set(0.0);
  } else {
    m_hand_right.Set(-dir);
    m_hand_left.Set(dir);
  }
}

void Arm::Drive(double low, double high) {
  m_goal_state = -2;
  m_low_actuator.Drive(low);
  m_high_actuator.Drive(high);
}

units::degree_t Arm::GetUpperAngle() { return m_high_encoder.GetNormalizedDistanceDegrees()-units::degree_t{ArmConstants::kArmHighOffset}; }
units::degree_t Arm::GetLowerAngle() { return m_low_encoder.GetNormalizedDistanceDegrees()-units::degree_t{ArmConstants::kArmLowOffset}; }
double Arm::GetRawUpperAngle() { return m_high_encoder.GetDistance(); }
double Arm::GetRawLowerAngle() { return m_low_encoder.GetDistance(); }

void Arm::SetClaw(int32_t claw_settings) {
  // if (claw_settings == 0) {

  // }
}