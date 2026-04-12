#include <Arduino.h>

#include "odometry.hpp"
#include "config.hpp"
#include "encoder_isr.hpp"
#include "imu_driver.hpp"
#include "shared_state.hpp"

namespace app {

void sensorUpdateTask(void*) {
    TickType_t last_wake = xTaskGetTickCount();
    std::uint32_t previous_us = micros();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));

        const std::uint32_t now_us = micros();
        float dt_seconds = static_cast<float>(now_us - previous_us) / 1000000.0f;
        previous_us = now_us;
        if (dt_seconds <= 0.0f || dt_seconds > 0.05f) {
            dt_seconds = static_cast<float>(SENSOR_PERIOD_MS) / 1000.0f;
        }

        IMUState imu_state = sharedState().getImuState();
        IMUState new_imu_state = imu_state;
        if (readImu(new_imu_state, dt_seconds)) {
            imu_state = new_imu_state;
            sharedState().setImuState(imu_state);
        }

        SensorState sensor_state = sharedState().getSensorState();
        const int raw_limit = digitalRead(LIMIT_SWITCH_PIN);
        sensor_state.limit_switch_pressed = LIMIT_SWITCH_ACTIVE_LOW ? (raw_limit == LOW) : (raw_limit == HIGH);
        sharedState().setSensorState(sensor_state);

        const std::array<std::int32_t, WHEEL_COUNT> encoder_counts = readEncoderCounts();
        const OdometryUpdateResult odom_update = odometryEstimator().update(encoder_counts, imu_state.orientation_z, dt_seconds);

        sharedState().setMeasuredWheelSpeeds(odom_update.wheel_rad_s);
        sharedState().setOdometryState(odom_update.odom);
    }
}

}  // namespace app
