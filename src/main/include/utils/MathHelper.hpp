#pragma once

#include <numbers>

inline double toRadians(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

inline double toDegrees(double radians) {
    return radians * 180.0 / std::numbers::pi;
}