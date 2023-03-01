// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/XboxController.h>

#include <numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

// TODO: Convert move of these to be config based rather than constexpr variables
// Some examples include:
//     . Button Mappings?
//     . Speeds, such as max total speed (as well as the mode swap speeds?)
//     . Slew rates that matter, such as kMagnitude and kRotational
//     . PID rates specifically for rotational
//     . Controller deadbands (which are the deadzones of the controller)

namespace ControlConstants {
    // Joystick Value [Currently Inline]

    // Fire While Held Button
    constexpr int xModeButton = frc::XboxController::Button::kX;

    // Fire Once Button
    constexpr int RelativeButton = frc::XboxController::Button::kY;
    constexpr int RateLimitButton = frc::XboxController::Button::kB;
    constexpr int DebugPrintButton = frc::XboxController::Button::kA;
    constexpr int SwapSpeedButton = frc::XboxController::Button::kLeftBumper;
    constexpr int AlignButton = frc::XboxController::Button::kRightBumper;
    constexpr int ResetHeadingButon = frc::XboxController::Button::kBack;
    constexpr int PosButton = frc::XboxController::Button::kStart;
}

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 4.8_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2 / 2 * std::numbers::pi};

// Mode Switch Speeds
constexpr units::meters_per_second_t FastSpeed = 3.0_mps;
constexpr units::meters_per_second_t LowSpeed = 1.0_mps;

constexpr double kDirectionSlewRate = 1.2;   // radians per second
// This controls the drive slew (daming on drive)
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
// This controls the damping on the robots rotation
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

// Chassis configuration
constexpr units::meter_t kTrackWidth =
    //0.6731_m;  // Distance between centers of right and left wheels on robot
    0.575_m;
constexpr units::meter_t kWheelBase =
    //0.6731_m;  // Distance between centers of front and back wheels on robot
    0.665_m;

// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2;
constexpr double kFrontRightChassisAngularOffset = 0;
constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2;

// SPARK MAX CAN IDs
constexpr int kFrontLeftDrivingCanId = 4;
constexpr int kRearLeftDrivingCanId = 6;
constexpr int kFrontRightDrivingCanId = 2;
constexpr int kRearRightDrivingCanId = 8;

constexpr int kFrontLeftTurningCanId = 3;
constexpr int kRearLeftTurningCanId = 5;
constexpr int kFrontRightTurningCanId = 1;
constexpr int kRearRightTurningCanId = 7;
}  // namespace DriveConstants

namespace ArmConstants {
constexpr int kHandLeftId = 0; // TODO!
constexpr int kHandRightId = 0; // TODO!
constexpr int kHandGrabId = 0; // TODO!
constexpr int kArmLowId = 0; // TODO!
constexpr int kArmHighId = 0; // TODO!

constexpr units::ampere_t kHandLeftCurrentLimit = 30_A; // TODO!
constexpr units::ampere_t kHandRightCurrentLimit = 30_A; // TODO!
constexpr units::ampere_t kHandGrabCurrentLimit = 30_A; // TODO!
constexpr units::ampere_t kArmLowCurrentLimit = 30_A; // TODO!
constexpr units::ampere_t kArmHighCurrentLimit = 30_A; // TODO!

constexpr double kArmLowP = 0.04; // TODO!
constexpr double kArmLowI = 0; // TODO!
constexpr double kArmLowD = 0; // TODO!
constexpr double kArmLowFF = 0; // TODO!

constexpr double kArmHighP = 0.04; // TODO!
constexpr double kArmHighI = 0; // TODO!
constexpr double kArmHighD = 0; // TODO!
constexpr double kArmHighFF = 0; // TODO!
} // namespace ArmConstants

namespace ModuleConstants {
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 12;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction =
    (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor =
    (kWheelDiameter.value() * std::numbers::pi) /
    kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
    ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
    60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor =
    (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
    (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
    units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.04;
constexpr double kDrivingI = 0;
constexpr double kDrivingD = 0;
constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 1;
constexpr double kTurningI = 0;
constexpr double kTurningD = 0;
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::CANSparkMax::IdleMode kDrivingMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;
constexpr rev::CANSparkMax::IdleMode kTurningMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 50_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr units::meter_t kAutoTargetX = -6.5_in;
constexpr units::meter_t kAutoTargetZ = 1_m;
constexpr units::meter_t kAutoTargetDeadzone = 1_in;
constexpr units::radian_t kAutoTargetAngularDeadzone = 2.5_deg;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;
}  // namespace AutoConstants

namespace IOConstants {
constexpr int kDriverControllerPort = 0;
constexpr double kDriveDeadband = 0.15;
}  // namespace IOConstants
