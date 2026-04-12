#pragma once

#include <cstdint>

namespace app {

bool initializeMicroRosTransport();
void shutdownMicroRosTransport();
void microrosSpinSome(std::uint32_t timeout_ms);
bool publishTelemetry();
bool microrosConnected();

}  // namespace app
