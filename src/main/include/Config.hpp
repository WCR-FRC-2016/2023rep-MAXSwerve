#pragma once

#include <fstream>
#include <filesystem>

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/velocity.h>

#include "vendor/json.hpp"
#include "frc/Filesystem.h"
#include "wpi/raw_ostream.h"

#include "Constants.h"

inline void loadConfig() {
    try {
        std::filesystem::path deploy_dir(frc::filesystem::GetDeployDirectory());

        std::ifstream config_file(deploy_dir.append("config.json"));

        nlohmann::json json = nlohmann::json::parse(config_file);
        // = json["test"].get<double>();
        DriveConstants::kMaxSpeed = units::meters_per_second_t(json["max-speed"].get<double>());
        DriveConstants::FastSpeed = units::meters_per_second_t(json["fast-speed"].get<double>());
        DriveConstants::LowSpeed = units::meters_per_second_t(json["slow-speed"].get<double>());

        frc::SmartDashboard::PutNumber("Config/Max Speed", DriveConstants::kMaxSpeed.value());
        frc::SmartDashboard::PutNumber("Config/Fast Speed", DriveConstants::FastSpeed.value());
        frc::SmartDashboard::PutNumber("Config/Slow Speed", DriveConstants::LowSpeed.value());

        config_file.close();
    } catch(...) {
        wpi::outs() << "Failed to load robot config!!!\n";
        wpi::outs().flush();
        throw; // I was told to make this crash the robot, to not fail gracefully.
    }
}