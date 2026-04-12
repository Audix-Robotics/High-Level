#include <Arduino.h>

#include <array>
#include <cmath>

#include "mecanum.hpp"
#include "motor_driver.hpp"
#include "pid.hpp"
#include "safety.hpp"
#include "shared_state.hpp"

namespace app {

void motionControlTask(void*) {
    std::array<PidController, WHEEL_COUNT> pid_controllers{};
    for (auto& controller : pid_controllers) {
        controller.configure(PID_KP, PID_KI, PID_KD, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
        controller.setIntegralClamp(-PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
    }

    MotionSafety safety;
    TickType_t last_wake = xTaskGetTickCount();
    bool previously_allowed = false;
    constexpr float dt_seconds = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

        const std::uint32_t now_ms = millis();
        const CommandState command = sharedState().getCommandState();
        const WheelState wheel_state = sharedState().getWheelState();
        const bool motion_allowed = safety.motionAllowed(command, now_ms);

        std::array<float, WHEEL_COUNT> targets{};
        if (motion_allowed) {
            const ChassisMotion requested_motion{command.cmd_vx, command.cmd_vy, command.cmd_wz};
            targets = inverseKinematics(requested_motion, ROBOT_GEOMETRY);
        }
        safety.enforce(command, now_ms, targets);
        sharedState().setTargetWheelSpeeds(targets);

        if (!motion_allowed && previously_allowed) {
            for (auto& controller : pid_controllers) {
                controller.reset();
            }
        }

        std::array<float, WHEEL_COUNT> outputs{};
        for (std::size_t index = 0; index < WHEEL_COUNT; ++index) {
            outputs[index] = pid_controllers[index].update(
                targets[index],
                wheel_state.measured_w_rad_s[index],
                dt_seconds,
                motion_allowed);

            if (!motion_allowed && std::fabs(wheel_state.measured_w_rad_s[index]) < 0.05f) {
                outputs[index] = 0.0f;
            }
            outputs[index] = clampFloat(outputs[index], -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
        }

        sharedState().setPwmOutputs(outputs);
        applyMotorOutputs(outputs);
        previously_allowed = motion_allowed;
    }
}

}  // namespace app
