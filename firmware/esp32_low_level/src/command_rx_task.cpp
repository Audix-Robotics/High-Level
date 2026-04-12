#include "shared_state.hpp"

#include "microros_transport.hpp"

namespace app {

void commandRxTask(void*) {
    for (;;) {
        microrosSpinSome(COMMAND_RX_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(COMMAND_RX_PERIOD_MS));
    }
}

}  // namespace app
