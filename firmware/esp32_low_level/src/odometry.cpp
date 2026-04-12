#include <cmath>

#include "config.hpp"
#include "odometry.hpp"

namespace app {

namespace {

OdometryEstimator g_odometry_estimator;

float shortestAngularDistance(float from, float to) {
    return wrapAngle(to - from);
}

}  // namespace

void OdometryEstimator::initialize(const std::array<std::int32_t, WHEEL_COUNT>& encoder_counts, float initial_yaw) {
    previous_counts_ = encoder_counts;
    odom_state_ = OdometryState{};
    odom_state_.theta = wrapAngle(initial_yaw);
    initialized_ = true;
}

OdometryUpdateResult OdometryEstimator::update(const std::array<std::int32_t, WHEEL_COUNT>& encoder_counts, float imu_yaw, float dt_seconds) {
    if (!initialized_) {
        initialize(encoder_counts, imu_yaw);
    }

    OdometryUpdateResult result{};
    if (dt_seconds <= 0.0f) {
        result.odom = odom_state_;
        return result;
    }

    WheelVector wheel_rad_s{};
    for (std::size_t index = 0; index < WHEEL_COUNT; ++index) {
        const std::int32_t delta_counts = encoder_counts[index] - previous_counts_[index];
        previous_counts_[index] = encoder_counts[index];

        const float delta_rad = static_cast<float>(delta_counts) * (TWO_PI_F / static_cast<float>(ENCODER_CPR)) * static_cast<float>(ENCODER_POLARITY[index]);
        wheel_rad_s[index] = delta_rad / dt_seconds;
        result.wheel_rad_s[index] = wheel_rad_s[index];
    }

    const ChassisMotion motion = forwardKinematics(wheel_rad_s, ROBOT_GEOMETRY);
    const float encoder_theta = wrapAngle(odom_state_.theta + motion.wz * dt_seconds);
    const float imu_error = shortestAngularDistance(encoder_theta, imu_yaw);
    const float correction = clampFloat(IMU_YAW_BLEND_GAIN * imu_error, -IMU_YAW_BLEND_MAX_STEP, IMU_YAW_BLEND_MAX_STEP);

    odom_state_.theta = wrapAngle(encoder_theta + correction);
    odom_state_.x += (motion.vx * std::cos(odom_state_.theta) - motion.vy * std::sin(odom_state_.theta)) * dt_seconds;
    odom_state_.y += (motion.vx * std::sin(odom_state_.theta) + motion.vy * std::cos(odom_state_.theta)) * dt_seconds;
    odom_state_.vx = motion.vx;
    odom_state_.vy = motion.vy;
    odom_state_.wtheta = motion.wz;

    result.odom = odom_state_;
    return result;
}

OdometryState OdometryEstimator::state() const {
    return odom_state_;
}

OdometryEstimator& odometryEstimator() {
    return g_odometry_estimator;
}

}  // namespace app
