#pragma once

#include <array>

#include "config.hpp"

namespace app {

struct ChassisMotion {
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
};

using WheelVector = std::array<float, WHEEL_COUNT>;

WheelVector inverseKinematics(const ChassisMotion& motion, const RobotGeometry& geometry);
ChassisMotion forwardKinematics(const WheelVector& wheel_rad_s, const RobotGeometry& geometry);

}  // namespace app
