#pragma once

#include <string>

#include "vendor/json.hpp"
#include "Logging.hpp"

template<typename T>
T getValueOrDefault(nlohmann::json json, std::string key, const T& default_value) {
    if (!json.contains(key)) {
        Logger::Log(LogLevel::Dev) << "Value is not in [" << key << "], returning default..." << LoggerCommand::Flush;
        return default_value;
    }

    return json[key].get<T>();
}