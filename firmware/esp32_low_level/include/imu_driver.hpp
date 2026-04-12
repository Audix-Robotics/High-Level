#pragma once

#include "shared_state.hpp"

namespace app {

bool initializeImu();
bool readImu(IMUState& state, float dt_seconds);
bool imuHealthy();

}  // namespace app
