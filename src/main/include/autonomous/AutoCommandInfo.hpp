#pragma once

#include <vector>

#include "Logging.hpp"
#include "vendor/json.hpp"

struct AutoCommandInfo {
    uint32_t CommandType;
    nlohmann::json CommandData;

    inline AutoCommandInfo(uint32_t command_type, nlohmann::json data) : CommandType(command_type), CommandData(data) { 
        //CommandData = nlohmann::json(data);
        //Logger::Log(LogLevel::Important) << "Json Command Data String: " << data << LoggerCommand::Flush;
        Logger::Log(LogLevel::Dev) << "Actual Data: " << CommandData.dump() << LoggerCommand::Flush;
    }
};