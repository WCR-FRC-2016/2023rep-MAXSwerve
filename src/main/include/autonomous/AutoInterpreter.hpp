#pragma once

#include <vector>

#include <frc2/command/SequentialCommandGroup.h>

#include "Logging.hpp"
#include "vendor/json.hpp"
#include "AutoSequence.hpp"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/Arm.h"
#include "subsystems/Limelight.h"
#include "subsystems/LEDController.h"

// Quick and dirty fix, change???
struct AutoSubsystemWrapper {
    DriveSubsystem& m_drive;
    Arm& m_arm;
    Limelight& m_limelight;
    LEDController& m_leds;

    AutoSubsystemWrapper(DriveSubsystem& drive, Arm& arm, Limelight& limelight, LEDController& leds) : 
        m_drive(drive), 
        m_arm(arm), 
        m_limelight(limelight), 
        m_leds(leds) { }
};

inline std::vector<AutoSequence> interpretJsonSequences(nlohmann::json& json) {
    std::vector<AutoSequence> sequences;

    for (auto& element : json) {
        std::vector<AutoCommandInfo> commands;

        auto& command_json = element["commands"];
        for (auto& command : command_json) {
            commands.emplace_back(command["type"].get<uint32_t>(), command["data"]);
        }

        sequences.emplace_back(element["name"].get<std::string>(), commands);
    }

    return sequences;
}

inline void mapAutoSequences() {
    for (auto i = 0u; i < AutoConstants::kAutoSequences.size(); i++)
        AutoConstants::kAutoSequencesMapped.emplace(AutoConstants::kAutoSequences[i].SequenceName, i);
}