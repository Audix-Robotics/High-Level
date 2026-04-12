#pragma once

#include <array>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "config.hpp"

namespace app {

struct CommandState {
    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_wz = 0.0f;
    bool robot_enabled = false;
    std::uint32_t last_cmd_time_ms = 0;
};

struct WheelState {
    std::array<float, WHEEL_COUNT> target_w_rad_s{};
    std::array<float, WHEEL_COUNT> measured_w_rad_s{};
    std::array<std::int32_t, WHEEL_COUNT> encoder_counts{};
    std::array<float, WHEEL_COUNT> pwm_output{};
};

struct OdometryState {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float wtheta = 0.0f;
};

struct IMUState {
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;
    float orientation_z = 0.0f;
};

struct SensorState {
    bool limit_switch_pressed = false;
};

class SharedStateStore {
public:
    void initialize() {
        if (command_mutex_ == nullptr) {
            command_mutex_ = xSemaphoreCreateMutex();
            wheel_mutex_ = xSemaphoreCreateMutex();
            odom_mutex_ = xSemaphoreCreateMutex();
            imu_mutex_ = xSemaphoreCreateMutex();
            sensor_mutex_ = xSemaphoreCreateMutex();
        }

        command_state_ = CommandState{};
        wheel_state_ = WheelState{};
        odom_state_ = OdometryState{};
        imu_state_ = IMUState{};
        sensor_state_ = SensorState{};
    }

    void setCommand(float vx, float vy, float wz, std::uint32_t now_ms) {
        lock(command_mutex_);
        command_state_.cmd_vx = vx;
        command_state_.cmd_vy = vy;
        command_state_.cmd_wz = wz;
        command_state_.last_cmd_time_ms = now_ms;
        unlock(command_mutex_);
    }

    void setRobotEnabled(bool enabled) {
        lock(command_mutex_);
        command_state_.robot_enabled = enabled;
        unlock(command_mutex_);
    }

    CommandState getCommandState() const {
        lock(command_mutex_);
        CommandState snapshot = command_state_;
        unlock(command_mutex_);
        return snapshot;
    }

    void setTargetWheelSpeeds(const std::array<float, WHEEL_COUNT>& values) {
        lock(wheel_mutex_);
        wheel_state_.target_w_rad_s = values;
        unlock(wheel_mutex_);
    }

    void setMeasuredWheelSpeeds(const std::array<float, WHEEL_COUNT>& values) {
        lock(wheel_mutex_);
        wheel_state_.measured_w_rad_s = values;
        unlock(wheel_mutex_);
    }

    void setPwmOutputs(const std::array<float, WHEEL_COUNT>& values) {
        lock(wheel_mutex_);
        wheel_state_.pwm_output = values;
        unlock(wheel_mutex_);
    }

    WheelState getWheelState() const {
        lock(wheel_mutex_);
        WheelState snapshot = wheel_state_;
        unlock(wheel_mutex_);

        portENTER_CRITICAL(&encoder_mux_);
        snapshot.encoder_counts = wheel_state_.encoder_counts;
        portEXIT_CRITICAL(&encoder_mux_);
        return snapshot;
    }

    void addEncoderDeltaFromIsr(WheelIndex wheel, std::int32_t delta) {
        portENTER_CRITICAL_ISR(&encoder_mux_);
        wheel_state_.encoder_counts[wheel] += delta;
        portEXIT_CRITICAL_ISR(&encoder_mux_);
    }

    std::array<std::int32_t, WHEEL_COUNT> getEncoderCounts() const {
        std::array<std::int32_t, WHEEL_COUNT> counts{};
        portENTER_CRITICAL(&encoder_mux_);
        counts = wheel_state_.encoder_counts;
        portEXIT_CRITICAL(&encoder_mux_);
        return counts;
    }

    void resetEncoderCounts() {
        portENTER_CRITICAL(&encoder_mux_);
        wheel_state_.encoder_counts = {};
        portEXIT_CRITICAL(&encoder_mux_);
    }

    void setOdometryState(const OdometryState& state) {
        lock(odom_mutex_);
        odom_state_ = state;
        unlock(odom_mutex_);
    }

    OdometryState getOdometryState() const {
        lock(odom_mutex_);
        OdometryState snapshot = odom_state_;
        unlock(odom_mutex_);
        return snapshot;
    }

    void setImuState(const IMUState& state) {
        lock(imu_mutex_);
        imu_state_ = state;
        unlock(imu_mutex_);
    }

    IMUState getImuState() const {
        lock(imu_mutex_);
        IMUState snapshot = imu_state_;
        unlock(imu_mutex_);
        return snapshot;
    }

    void setSensorState(const SensorState& state) {
        lock(sensor_mutex_);
        sensor_state_ = state;
        unlock(sensor_mutex_);
    }

    SensorState getSensorState() const {
        lock(sensor_mutex_);
        SensorState snapshot = sensor_state_;
        unlock(sensor_mutex_);
        return snapshot;
    }

private:
    static void lock(SemaphoreHandle_t mutex) {
        if (mutex != nullptr) {
            xSemaphoreTake(mutex, portMAX_DELAY);
        }
    }

    static void unlock(SemaphoreHandle_t mutex) {
        if (mutex != nullptr) {
            xSemaphoreGive(mutex);
        }
    }

    mutable SemaphoreHandle_t command_mutex_ = nullptr;
    mutable SemaphoreHandle_t wheel_mutex_ = nullptr;
    mutable SemaphoreHandle_t odom_mutex_ = nullptr;
    mutable SemaphoreHandle_t imu_mutex_ = nullptr;
    mutable SemaphoreHandle_t sensor_mutex_ = nullptr;
    mutable portMUX_TYPE encoder_mux_ = portMUX_INITIALIZER_UNLOCKED;

    CommandState command_state_{};
    WheelState wheel_state_{};
    OdometryState odom_state_{};
    IMUState imu_state_{};
    SensorState sensor_state_{};
};

SharedStateStore& sharedState();

void commandRxTask(void* parameters);
void motionControlTask(void* parameters);
void sensorUpdateTask(void* parameters);
void telemetryTask(void* parameters);

}  // namespace app
