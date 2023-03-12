#pragma once

#include <vector>

#include "vendor/json.hpp"

struct AutoCommandInfo {
    uint32_t CommandType;
    nlohmann::json CommandData;

    inline AutoCommandInfo(uint32_t command_type, nlohmann::json data) : CommandType(command_type), CommandData(data) { }
};