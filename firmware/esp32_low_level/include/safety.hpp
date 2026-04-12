#pragma once

#include <array>
#include <cstdint>

#include "config.hpp"
#include "shared_state.hpp"

namespace app {

class MotionSafety {
public:
    bool motionAllowed(const CommandState& command, std::uint32_t now_ms) const;
    void enforce(const CommandState& command, std::uint32_t now_ms, std::array<float, WHEEL_COUNT>& targets) const;
};

}  // namespace app
