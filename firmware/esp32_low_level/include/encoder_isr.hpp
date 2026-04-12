#pragma once

#include <array>
#include <cstdint>

#include "config.hpp"

namespace app {

void initializeEncoders();
std::array<std::int32_t, WHEEL_COUNT> readEncoderCounts();
void resetEncoderCounts();

void IRAM_ATTR onFrontLeftEncoderEdge();
void IRAM_ATTR onFrontRightEncoderEdge();
void IRAM_ATTR onRearLeftEncoderEdge();
void IRAM_ATTR onRearRightEncoderEdge();

}  // namespace app
