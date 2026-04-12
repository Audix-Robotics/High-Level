#include <Arduino.h>

#include "config.hpp"
#include "encoder_isr.hpp"
#include "imu_driver.hpp"
#include "microros_transport.hpp"
#include "motor_driver.hpp"
#include "odometry.hpp"
#include "shared_state.hpp"

namespace app {

namespace {

SharedStateStore g_shared_state;

}  // namespace

SharedStateStore& sharedState() {
    return g_shared_state;
}

}  // namespace app

void setup() {
    Serial.begin(115200);
    delay(250);

    app::sharedState().initialize();
    pinMode(app::LIMIT_SWITCH_PIN, app::LIMIT_SWITCH_ACTIVE_LOW ? INPUT_PULLUP : INPUT);

    app::initializeEncoders();
    app::initializeMotorDriver();
    app::initializeImu();

    const std::array<std::int32_t, app::WHEEL_COUNT> encoder_counts = app::readEncoderCounts();
    app::odometryEstimator().initialize(encoder_counts, 0.0f);
    app::initializeMicroRosTransport();

    xTaskCreatePinnedToCore(app::commandRxTask, "command_rx", app::COMMAND_RX_TASK_STACK, nullptr, app::COMMAND_RX_TASK_PRIORITY, nullptr, app::COMMAND_RX_CORE);
    xTaskCreatePinnedToCore(app::motionControlTask, "motion_control", app::MOTION_CONTROL_TASK_STACK, nullptr, app::MOTION_CONTROL_TASK_PRIORITY, nullptr, app::MOTION_CONTROL_CORE);
    xTaskCreatePinnedToCore(app::sensorUpdateTask, "sensor_update", app::SENSOR_TASK_STACK, nullptr, app::SENSOR_TASK_PRIORITY, nullptr, app::SENSOR_CORE);
    xTaskCreatePinnedToCore(app::telemetryTask, "telemetry", app::TELEMETRY_TASK_STACK, nullptr, app::TELEMETRY_TASK_PRIORITY, nullptr, app::TELEMETRY_CORE);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
