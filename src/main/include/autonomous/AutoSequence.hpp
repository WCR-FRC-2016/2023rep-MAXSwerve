#pragma once

#include <string>
#include <vector>

#include "AutoCommandInfo.hpp"

struct AutoSequence {
    std::string SequenceName;
    std::vector<AutoCommandInfo> Commands;
};