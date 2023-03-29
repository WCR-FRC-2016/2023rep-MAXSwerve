#pragma once

#include <fstream>
#include <filesystem>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <units/velocity.h>

#include "vendor/json.hpp"
#include "frc/Filesystem.h"
#include "wpi/raw_ostream.h"

#include "autonomous/AutoInterpreter.hpp"
#include "Constants.h"
#include "Logging.hpp"

inline void loadConfig() {
    try {
        std::filesystem::path deploy_dir(frc::filesystem::GetDeployDirectory());

        std::ifstream config_file(deploy_dir.append("config.json"));

        nlohmann::json json = nlohmann::json::parse(config_file);
        // = json["test"].get<double>();

        // Logging
        // Ensure None is not true, if none dont set any log levels!
        if (!json["log-levels"]["none"].get<bool>()) {
            uint32_t log_level = 0;

            auto levels = json["log-levels"];
            if (levels["info"].get<bool>())      log_level |= LogLevel::Info;
            if (levels["utility"].get<bool>())   log_level |= LogLevel::Utility;
            if (levels["important"].get<bool>()) log_level |= LogLevel::Important;
            if (levels["error"].get<bool>())     log_level |= LogLevel::Error;
            if (levels["match"].get<bool>())     log_level |= LogLevel::Match;
            if (levels["dev"].get<bool>())       log_level |= LogLevel::Dev;

            Logger::SetGlobalLevel(log_level);
        } else Logger::SetGlobalLevel(LogLevel::None);

        if (json["log-levels"]["all"]) Logger::SetGlobalLevel(LogLevel::All);

        // Drive Constants
        DriveConstants::kDriveMaxSpeed = units::meters_per_second_t(json["drive-max-speed"].get<double>());
        DriveConstants::kFastSpeed = units::meters_per_second_t(json["fast-speed"].get<double>());
        DriveConstants::kLowSpeed = units::meters_per_second_t(json["slow-speed"].get<double>());
        DriveConstants::kMoveOverTime = json["move-over-time"].get<double>();
        DriveConstants::kMoveOverSubTime = json["move-over-sub-time"].get<double>();
        DriveConstants::kTargetBalanceRoll = json["target-balance-roll"].get<double>();
        DriveConstants::kDefaultSlow = json["default-slow"].get<bool>();

        auto max_rot_speed = json["drive-max-rot-speed"].get<double>();
        auto max_rot_speed_rads = max_rot_speed * std::numbers::pi / 180.0;

        //auto fast_rot_speed = json[""]

        DriveConstants::kMaxAngularSpeed = units::radians_per_second_t{max_rot_speed_rads};

        Logger::Log(LogLevel::Dev) << "Max Rot Speed: " << max_rot_speed_rads << LoggerCommand::Flush;

        // Auto Constants
        AutoConstants::kAutoMaxSpeed = units::meters_per_second_t{json["auto-max-speed"].get<double>()};
        AutoConstants::kPXController = json["auto-pcontroller-x"].get<double>();
        AutoConstants::kPYController = json["auto-pcontroller-y"].get<double>();
        AutoConstants::kPThetaController = json["auto-pcontroller-theta"].get<double>();
        AutoConstants::kAlignSpeed = json["align-speed"].get<double>();
        AutoConstants::kAlignRotationSpeed = json["align-rotation-speed"].get<double>();

        // Autonomous Routines
        AutoConstants::kAutoSequences = interpretJsonSequences(json["autonomous-routines"]);
        //mapAutoSequences();

        // Set up autonomous command chooser
        std::vector<std::string> autoVector {};
        for (unsigned int i = 0; i < AutoConstants::kAutoSequences.size(); i++) {
            std::string name = AutoConstants::kAutoSequences[i].SequenceName;
            autoVector.push_back(name);
        }
        frc::SmartDashboard::PutStringArray("Auto List", autoVector);

        // Arm Constants
        ArmConstants::kArmLowOffset  = json["arm-low-offset"].get<double>();
        ArmConstants::kArmHighOffset = json["arm-high-offset"].get<double>();
        ArmConstants::kArmLowP  = json["arm-low-p"].get<double>();
        ArmConstants::kArmLowI  = json["arm-low-i"].get<double>();
        ArmConstants::kArmLowD  = json["arm-low-d"].get<double>();
        ArmConstants::kArmHighP = json["arm-high-p"].get<double>();
        ArmConstants::kArmHighI = json["arm-high-i"].get<double>();
        ArmConstants::kArmHighD = json["arm-high-d"].get<double>();
        ArmConstants::kClawMoveTime = json["claw-move-time"].get<double>();
        ArmConstants::kUseLimitSwitches = json["use-limit-switches"].get<bool>();
        ArmConstants::kUseTiming = json["use-timing"].get<bool>();

        // Other Stuff
        IOConstants::kLEDBrightness = json["led-brightness"].get<double>();


        frc::SmartDashboard::PutNumber("Config/Max Speed", DriveConstants::kMaxSpeed.value());
        frc::SmartDashboard::PutNumber("Config/Fast Speed", DriveConstants::kFastSpeed.value());
        frc::SmartDashboard::PutNumber("Config/Slow Speed", DriveConstants::kLowSpeed.value());

        config_file.close();
    } catch(std::exception& ec) {
        wpi::outs() << "Failed to load robot config!!!\n";
        wpi::outs() << ec.what() << "\n";
        wpi::outs().flush();
        throw; // I was told to make this crash the robot, to not fail gracefully.
    }
}