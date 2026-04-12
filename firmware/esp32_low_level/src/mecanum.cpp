#include "mecanum.hpp"

namespace app {

WheelVector inverseKinematics(const ChassisMotion& motion, const RobotGeometry& geometry) {
    const float k = geometry.wheelbase_half_m + geometry.track_half_width_m;
    const float inv_r = 1.0f / geometry.wheel_radius_m;

    WheelVector wheel_rad_s{};
    wheel_rad_s[FL] = inv_r * (motion.vx + motion.vy - k * motion.wz);
    wheel_rad_s[FR] = inv_r * (motion.vx - motion.vy + k * motion.wz);
    wheel_rad_s[RL] = inv_r * (motion.vx - motion.vy - k * motion.wz);
    wheel_rad_s[RR] = inv_r * (motion.vx + motion.vy + k * motion.wz);
    return wheel_rad_s;
}

ChassisMotion forwardKinematics(const WheelVector& wheel_rad_s, const RobotGeometry& geometry) {
    const float r = geometry.wheel_radius_m;
    const float k = geometry.wheelbase_half_m + geometry.track_half_width_m;

    ChassisMotion motion{};
    motion.vx = (r / 4.0f) * (wheel_rad_s[FL] + wheel_rad_s[FR] + wheel_rad_s[RL] + wheel_rad_s[RR]);
    motion.vy = (r / 4.0f) * (-wheel_rad_s[FL] + wheel_rad_s[FR] + wheel_rad_s[RL] - wheel_rad_s[RR]);
    motion.wz = (r / (4.0f * k)) * (-wheel_rad_s[FL] + wheel_rad_s[FR] - wheel_rad_s[RL] + wheel_rad_s[RR]);
    return motion;
}

}  // namespace app
