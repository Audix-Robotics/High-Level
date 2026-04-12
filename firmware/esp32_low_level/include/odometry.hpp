#pragma once

#include <array>

#include "mecanum.hpp"
#include "shared_state.hpp"

namespace app {

struct OdometryUpdateResult {
    std::array<float, WHEEL_COUNT> wheel_rad_s{};
    OdometryState odom{};
};

class OdometryEstimator {
public:
    void initialize(const std::array<std::int32_t, WHEEL_COUNT>& encoder_counts, float initial_yaw);
    OdometryUpdateResult update(const std::array<std::int32_t, WHEEL_COUNT>& encoder_counts, float imu_yaw, float dt_seconds);
    OdometryState state() const;

private:
    std::array<std::int32_t, WHEEL_COUNT> previous_counts_{};
    OdometryState odom_state_{};
    bool initialized_ = false;
};

OdometryEstimator& odometryEstimator();

}  // namespace app
