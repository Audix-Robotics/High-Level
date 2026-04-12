#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <freertos/FreeRTOS.h>

namespace app {

constexpr std::size_t WHEEL_COUNT = 4;

enum WheelIndex : std::size_t {
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3,
};

struct RobotGeometry {
    float wheel_radius_m;
    float track_half_width_m;
    float wheelbase_half_m;
};

struct MotorPinConfig {
    std::uint8_t input_a_pin;
    std::uint8_t input_b_pin;
    std::uint8_t input_a_channel;
    std::uint8_t input_b_channel;
};

struct EncoderPinConfig {
    std::uint8_t channel_a_pin;
    std::uint8_t channel_b_pin;
};

constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 6.28318530717958647692f;
constexpr float GRAVITY_M_S2 = 9.80665f;

constexpr RobotGeometry ROBOT_GEOMETRY{
    0.0485f,
    0.1574f,
    0.090f,
};

constexpr std::int32_t ENCODER_CPR = 1320;

constexpr std::uint32_t COMMAND_RX_PERIOD_MS = 1;
constexpr std::uint32_t CONTROL_PERIOD_MS = 10;
constexpr std::uint32_t SENSOR_PERIOD_MS = 5;
constexpr std::uint32_t TELEMETRY_PERIOD_MS = 20;
constexpr std::uint32_t CMD_TIMEOUT_MS = 500;

constexpr std::uint32_t COMMAND_RX_TASK_PRIORITY = 4;
constexpr std::uint32_t MOTION_CONTROL_TASK_PRIORITY = 5;
constexpr std::uint32_t SENSOR_TASK_PRIORITY = 3;
constexpr std::uint32_t TELEMETRY_TASK_PRIORITY = 2;

constexpr std::uint32_t COMMAND_RX_TASK_STACK = 8192;
constexpr std::uint32_t MOTION_CONTROL_TASK_STACK = 8192;
constexpr std::uint32_t SENSOR_TASK_STACK = 6144;
constexpr std::uint32_t TELEMETRY_TASK_STACK = 8192;

constexpr BaseType_t COMMAND_RX_CORE = 0;
constexpr BaseType_t MOTION_CONTROL_CORE = 1;
constexpr BaseType_t SENSOR_CORE = 0;
constexpr BaseType_t TELEMETRY_CORE = 0;

constexpr std::int32_t PWM_MAX = 255;
constexpr std::uint32_t PWM_FREQUENCY = 20000;
constexpr std::uint8_t PWM_RESOLUTION_BITS = 8;
constexpr float PID_KP = 1.5f;
constexpr float PID_KI = 0.8f;
constexpr float PID_KD = 0.05f;
constexpr float PID_INTEGRAL_MAX = 100.0f;
constexpr float PID_OUTPUT_MAX = 255.0f;
constexpr float MOTOR_DEADBAND = 18.0f;

constexpr float IMU_YAW_BLEND_GAIN = 0.15f;
constexpr float IMU_YAW_BLEND_MAX_STEP = 0.03f;

constexpr std::uint8_t IMU_I2C_ADDRESS = 0x68;
constexpr std::uint8_t IMU_SDA_PIN = 21;
constexpr std::uint8_t IMU_SCL_PIN = 22;
constexpr std::uint32_t IMU_I2C_FREQUENCY = 400000;

constexpr std::uint8_t LIMIT_SWITCH_PIN = 23;
constexpr bool LIMIT_SWITCH_ACTIVE_LOW = true;

// Wheel-order assumption for the current firmware mapping:
// encoder 1 and motor A -> FL, encoder 2 and motor B -> FR,
// encoder 3 and motor C -> RL, encoder 4 and motor D -> RR.
// Verify this against the physical robot before deployment.
constexpr std::array<MotorPinConfig, WHEEL_COUNT> MOTOR_PINS{{
    {27, 14, 0, 1},
    {13, 19, 2, 3},
    {4, 16, 4, 5},
    {17, 18, 6, 7},
}};

constexpr std::array<EncoderPinConfig, WHEEL_COUNT> ENCODER_PINS{{
    {39, 36},
    {34, 35},
    {33, 32},
    {25, 26},
}};

constexpr std::array<int, WHEEL_COUNT> MOTOR_POLARITY{{1, 1, 1, 1}};
constexpr std::array<int, WHEEL_COUNT> ENCODER_POLARITY{{1, 1, 1, 1}};
constexpr std::array<const char*, WHEEL_COUNT> WHEEL_LABELS{{"FL", "FR", "RL", "RR"}};

inline float wrapAngle(float angle_rad) {
    while (angle_rad > PI_F) {
        angle_rad -= TWO_PI_F;
    }
    while (angle_rad < -PI_F) {
        angle_rad += TWO_PI_F;
    }
    return angle_rad;
}

inline float clampFloat(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

}  // namespace app
