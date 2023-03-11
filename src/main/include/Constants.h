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
#include <frc/trajectory/Trajectory.h>

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
    constexpr int xModeButton    = frc::XboxController::Button::kX;
    constexpr int DebugLEDButton = frc::XboxController::Button::kX;

    // Fire Once Button
    constexpr int RelativeButton     = frc::XboxController::Button::kY;
    constexpr int RateLimitButton    = frc::XboxController::Button::kB;
    constexpr int DebugPrintButton   = frc::XboxController::Button::kA;
    constexpr int SwapSpeedButton    = frc::XboxController::Button::kLeftBumper;
    constexpr int AlignButton        = frc::XboxController::Button::kRightBumper;
    constexpr int ResetHeadingButton = frc::XboxController::Button::kBack;
    constexpr int PosButton          = frc::XboxController::Button::kStart;
    
    constexpr int PosCarryButton = frc::XboxController::Button::kA;
    constexpr int PosMedButton   = frc::XboxController::Button::kB;
    constexpr int PosHighButton  = frc::XboxController::Button::kY;
    constexpr int PosSubButton   = frc::XboxController::Button::kX;
}

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
inline units::meters_per_second_t kMaxSpeed = 4.8_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2 / 2 * std::numbers::pi};

// Mode Switch Speeds
inline units::meters_per_second_t kFastSpeed = 3.0_mps;
inline units::meters_per_second_t kLowSpeed = 1.0_mps;

constexpr double kDirectionSlewRate = 1.2;   // radians per second
// This controls the drive slew (damping on drive)
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
constexpr double kFrontLeftChassisAngularOffset  = -std::numbers::pi / 2;
constexpr double kFrontRightChassisAngularOffset = 0;
constexpr double kRearLeftChassisAngularOffset   = std::numbers::pi;
constexpr double kRearRightChassisAngularOffset  = std::numbers::pi / 2;

// SPARK MAX CAN IDs
constexpr int kFrontLeftDrivingCanId  = 15;
constexpr int kRearLeftDrivingCanId   = 14;
constexpr int kRearRightDrivingCanId  = 13;
constexpr int kFrontRightDrivingCanId = 12;

constexpr int kFrontLeftTurningCanId  = 11;
constexpr int kRearLeftTurningCanId   = 10;
constexpr int kRearRightTurningCanId  = 9;
constexpr int kFrontRightTurningCanId = 8;
}  // namespace DriveConstants

namespace ArmConstants {
constexpr int kHandLeftId  = 0; // TODO!
constexpr int kHandRightId = 0; // TODO!
constexpr int kHandGrabId  = 0; // TODO!
constexpr int kArmLowId    = 0; // TODO!
constexpr int kArmHighId   = 0; // TODO!

constexpr int kArmLowEncoderId  = 0; // DIO slot 0 on RoboRio 2.0
constexpr int kArmHighEncoderId = 1; // DIO slot 1 on RoboRio 2.0

constexpr units::ampere_t kHandLeftCurrentLimit  = 26_A; // Works between 2 and 26 amps [Johnson Electric PLG Motor -> https://www.andymark.com/products/johnson-electric-gearmotor-and-output-shaft]
constexpr units::ampere_t kHandRightCurrentLimit = 26_A; // Works between 2 and 26 amps [Johnson Electric PLG Motor -> https://www.andymark.com/products/johnson-electric-gearmotor-and-output-shaft]
constexpr units::ampere_t kHandGrabCurrentLimit  = 26_A; // Works between 2 and 26 amps [Johnson Electric PLG Motor -> https://www.andymark.com/products/johnson-electric-gearmotor-and-output-shaft]
constexpr units::ampere_t kArmLowCurrentLimit    = 3_A;  // Very hard to find, max found on amazon listing [ECO-WORTHY 14mm/s 1000N -> https://www.amazon.com/ECO-LLC-Actuator-Mounting-Brackets/dp/B07L7XCSDW/ref=sr_1_2?c=ts&keywords=Linear+Motion+Actuators&qid=1678491620&refinements=p_89%3AECO-WORTHY&s=industrial&sr=1-2-catcorr&ts_id=350654011]
constexpr units::ampere_t kArmHighCurrentLimit   = 12_A; // Works between 4 and 12 amps [PA-04 -> https://www.progressiveautomations.com/products/linear-actuator-ip66]

constexpr double kArmLowP = 0.04; // TODO!
constexpr double kArmLowI = 0; // TODO!
constexpr double kArmLowD = 0; // TODO!

constexpr double kArmHighP = 0.04; // TODO!
constexpr double kArmHighI = 0; // TODO!
constexpr double kArmHighD = 0; // TODO!
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

// Still may need to be adjusted
constexpr double kDrivingP  = 0.04;
constexpr double kDrivingI  = 0;
constexpr double kDrivingD  = 0;
constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

// Still may need to be adjusted
constexpr double kTurningP  = 1;
constexpr double kTurningI  = 0;
constexpr double kTurningD  = 0;
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
inline frc::Trajectory kAutoTrajectory;

constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr units::meter_t kAutoTargetX = -6.5_in;
constexpr units::meter_t kAutoTargetZ = 1_m;
constexpr units::meter_t kAutoTargetDeadzone = 1_in;
constexpr units::radian_t kAutoTargetAngularDeadzone = 2.5_deg;

// TEMPORARY INLINE
inline double kPXController = 0.5;
inline double kPYController = 0.5;
inline double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;
}  // namespace AutoConstants

namespace IOConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kManipControllerPort = 1;
constexpr double kDriveDeadband = 0.15;
}  // namespace IOConstants
