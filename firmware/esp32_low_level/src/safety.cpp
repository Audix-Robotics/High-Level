#include "safety.hpp"

namespace app {

bool MotionSafety::motionAllowed(const CommandState& command, std::uint32_t now_ms) const {
    if (!command.robot_enabled) {
        return false;
    }
    return (now_ms - command.last_cmd_time_ms) < CMD_TIMEOUT_MS;
}

void MotionSafety::enforce(const CommandState& command, std::uint32_t now_ms, std::array<float, WHEEL_COUNT>& targets) const {
    if (!motionAllowed(command, now_ms)) {
        targets.fill(0.0f);
    }
}

}  // namespace app
