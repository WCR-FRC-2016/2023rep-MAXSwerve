#pragma once

#include <vector>

#include <frc2/command/SequentialCommandGroup.h>

#include "vendor/json.hpp"
#include "AutoSequence.hpp"
#include "autonomous/commands/AutoMoveCommand.hpp"

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

inline frc2::SequentialCommandGroup* createAutonomousCommandGroup(AutoSubsystemWrapper& wrapper, AutoSequence& sequence) {
    //frc2::SequentialCommandGroup* group = new frc2::SequentialCommandGroup();
    //std::vector<std::unique_ptr<frc2::Command>> sequence_commands;

    for (auto& command : sequence.Commands) {
        switch(command.CommandType) {
            case 0:
                //sequence_commands.push_back(std::make_unique<frc2::Command>(new AutoMoveCommand(wrapper, command)));
                break;
            default:
                break;
        }
    }

    return nullptr; 
}