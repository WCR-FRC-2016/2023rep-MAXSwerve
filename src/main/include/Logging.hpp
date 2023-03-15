#pragma once

#include <string>
#include <sstream>
#include "wpi/raw_ostream.h"

#include "units/length.h"
#include "units/angle.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"
#include "frc/geometry/Transform2d.h"

enum LogLevel : uint32_t
{
    None = 0,
    Info = 1,
    Utility = 1 << 1,
    Important = 1 << 2,
    Error = 1 << 3,
    Dev = 1 << 4,
    Autonomous = 1 << 5,
    All = UINT_MAX
};

enum class LoggerCommand
{
    Flush // Adds "\n" and wpi::outs().flush()'s
};

class Logger
{
public:
    Logger &operator<<(const std::string &);
    Logger &operator<<(const bool &);
    Logger &operator<<(const int &);
    Logger &operator<<(const float &);
    Logger &operator<<(const double &);
    Logger &operator<<(const char &);
    Logger &operator<<(const LoggerCommand &);

    Logger &operator<<(const units::meter_t &);
    Logger &operator<<(const units::radian_t &);
    Logger &operator<<(const units::degree_t &);
    Logger &operator<<(const units::meters_per_second_t &);
    Logger &operator<<(const units::radians_per_second_t &);
    Logger &operator<<(const units::degrees_per_second_t &);
    Logger &operator<<(const frc::Translation2d &translation);

private:
    uint32_t m_global_level;

    uint32_t m_level = 0;
    bool m_new_line = false;
    bool m_construct_name = true;

    Logger() {}

    static Logger &getLogger()
    {
        static Logger logger;
        return logger;
    }

    static std::string getMappedValue(const uint32_t value)
    {
        switch (value)
        {
        case 0:
            return "None";
        case 1:
            return "Info";
        case 1 << 1:
            return "Utility";
        case 1 << 2:
            return "Important";
        case 1 << 3:
            return "Error";
        case 1 << 4:
            return "Dev";
        case 1 << 5:
            return "Autonomous";
        default:
            return "Invalid";
        }
    }

    std::string constructName();
    bool checkLevel();

public:
    // Logging function, sets the message output channels and returns a logger to push values into
    static Logger &Log(const uint32_t log_level)
    {

        auto &logger = getLogger();
        logger.m_level = log_level;
        logger.m_new_line = true;

        return logger;
    }
    // Logging function, sets the message output channels and returns a logger to push values into
    static Logger &Log(const LogLevel log_level)
    {
        auto &logger = getLogger();
        logger.m_level = log_level;
        logger.m_new_line = true;

        return logger;
    }
    // Logging function with the ability to set whether it adds the levels names
    static Logger &Log(const LogLevel log_level, const bool construct_name)
    {
        auto &logger = getLogger();
        logger.m_level = log_level;
        logger.m_new_line = true;
        logger.m_construct_name = construct_name;

        return logger;
    }
    // Logging function with the ability to set whether it adds the levels names
    static Logger &Log(const uint32_t log_level, const bool construct_name)
    {

        auto &logger = getLogger();
        logger.m_level = log_level;
        logger.m_new_line = true;
        logger.m_construct_name = construct_name;

        return logger;
    }

    // Sets the global logging level (and what channels it accepts, use | to add multiple)
    static void SetGlobalLevel(const int level)
    {
        getLogger().m_global_level = level;
    }
    // Sets the global logging level (and what channels it accepts, use | to add multiple)
    static void SetGlobalLevel(const LogLevel log_level)
    {
        getLogger().m_global_level = log_level;
    }

    // Sets whether it should include the levels being logged [If you constantly use, make this false] (enabled by default)
    static void SetConstructName(const bool value)
    {
        getLogger().m_construct_name = value;
    }
};