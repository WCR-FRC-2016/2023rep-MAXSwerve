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
            if (levels["dev"].get<bool>())       log_level |= LogLevel::Dev;

            Logger::SetGlobalLevel(log_level);
        } else Logger::SetGlobalLevel(LogLevel::None);

        // Drive Constants
        DriveConstants::kMaxSpeed = units::meters_per_second_t(json["max-speed"].get<double>());
        DriveConstants::kFastSpeed = units::meters_per_second_t(json["fast-speed"].get<double>());
        DriveConstants::kLowSpeed = units::meters_per_second_t(json["slow-speed"].get<double>());

        // Auto Constants
        AutoConstants::kPXController = json["auto-pcontroller-x"].get<double>();
        AutoConstants::kPYController = json["auto-pcontroller-y"].get<double>();
        AutoConstants::kPThetaController = json["auto-pcontroller-theta"].get<double>();
        
        frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);

        switch (json["auto-command"].get<int>()) {                
            case 1:
                AutoConstants::kAutoTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                    {
                        frc::Pose2d{0_m, 0_m, 0_deg},
                        frc::Pose2d(1.0_m, 0.0_m, 0_deg)
                    },
                    config);
                break;
            case 2:
                AutoConstants::kAutoTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                    {
                        frc::Pose2d{0_m, 0_m, 0_deg},
                        frc::Pose2d(1.0_m, 0.0_m, 90_deg)
                    },
                    config);
                break;
            case 0:
            default:
                    AutoConstants::kAutoTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                    {
                        frc::Pose2d{0_m, 0_m, 0_deg},
                        frc::Pose2d(0.0_m, 0.0_m, 90_deg)
                    },
                    config);
                break;
        }

        frc::SmartDashboard::PutNumber("Config/Max Speed", DriveConstants::kMaxSpeed.value());
        frc::SmartDashboard::PutNumber("Config/Fast Speed", DriveConstants::kFastSpeed.value());
        frc::SmartDashboard::PutNumber("Config/Slow Speed", DriveConstants::kLowSpeed.value());

        config_file.close();
    } catch(...) {
        wpi::outs() << "Failed to load robot config!!!\n";
        wpi::outs().flush();
        throw; // I was told to make this crash the robot, to not fail gracefully.
    }
}