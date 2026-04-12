#pragma once

#include <array>

#include "config.hpp"

namespace app {

void initializeMotorDriver();
void applyMotorOutputs(const std::array<float, WHEEL_COUNT>& pwm_outputs);
void stopAllMotors();

}  // namespace app
