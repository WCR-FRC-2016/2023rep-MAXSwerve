#include "Logging.hpp"

bool Logger::checkLevel()
{
    const auto log = (m_global_level & m_level) != 0;

    if (log && m_new_line && m_construct_name)
    {
        wpi::outs() << "[" << constructName() << "]: ";
        m_new_line = false;
    }

    return log;
}

// Internal function for constructing channels
std::string Logger::constructName()
{
    std::stringstream value;
    auto levels = (m_global_level & m_level);
    uint32_t level_count = 0;

    for (auto i = 0; i < 32; i++)
    {
        if ((levels & (1 << i)) != 0)
        {
            if (level_count != 0)
                value << " | ";

            value << getMappedValue(1 << i);
            level_count++;
        }
    }

    return value.str();
}

Logger &Logger::operator<<(const std::string &message)
{
    if (!checkLevel())
        return *this;
    wpi::outs() << message;
    return *this;
}

Logger &Logger::operator<<(const int &number)
{
    if (!checkLevel())
        return *this;
    wpi::outs() << std::to_string(number);
    return *this;
}

Logger &Logger::operator<<(const float &number)
{
    if (!checkLevel())
        return *this;
    wpi::outs() << std::to_string(number);
    return *this;
}

Logger &Logger::operator<<(const double &number)
{
    if (!checkLevel())
        return *this;
    wpi::outs() << std::to_string(number);
    return *this;
}

Logger &Logger::operator<<(const char &number)
{
    if (!checkLevel())
        return *this;
    wpi::outs() << std::to_string(number);
    return *this;
}

Logger &Logger::operator<<(const LoggerCommand &command)
{
    if (!checkLevel())
        return *this;

    if (command == LoggerCommand::Flush)
    {
        wpi::outs() << "\n";
        wpi::outs().flush();
    }
    return *this;
}


Logger &Logger::operator<<(const units::meter_t &number)
{
    if (!checkLevel())
        return *this;
    wpi::outs() << std::to_string(number.value()) << " " << number.abbreviation();
    return *this;
}
Logger &Logger::operator<<(const units::radian_t &number)
{
    if (!checkLevel())
        return *this;
    wpi::outs() << std::to_string(number.value()) << " " << number.abbreviation();
    return *this;
}
Logger &Logger::operator<<(const units::degree_t &number)
{
    if (!checkLevel())
        return *this;
    wpi::outs() << std::to_string(number.value()) << " " << number.abbreviation();
    return *this;
}
Logger &Logger::operator<<(const units::meters_per_second_t &number)
{
    if (!checkLevel())
        return *this;
    wpi::outs() << std::to_string(number.value()) << " " << number.abbreviation();
    return *this;
}
Logger &Logger::operator<<(const units::radians_per_second_t &number)
{
    if (!checkLevel())
        return *this;
    wpi::outs() << std::to_string(number.value()) << " " << number.abbreviation();
    return *this;
}
Logger &Logger::operator<<(const units::degrees_per_second_t &number)
{
    if (!checkLevel())
        return *this;
    wpi::outs() << std::to_string(number.value()) << " " << number.abbreviation();
    return *this;
}
Logger &Logger::operator<<(const frc::Translation2d &translation) 
{
    if (!checkLevel())
        return *this;

    (*this) << "[Translation2D, " << translation.X() << " x, " << translation.Y() << " y]";
    return *this;
}