#include "shared_state.hpp"

#include "config.hpp"
#include "microros_transport.hpp"

namespace app {

void telemetryTask(void*) {
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        publishTelemetry();
    }
}

}  // namespace app
