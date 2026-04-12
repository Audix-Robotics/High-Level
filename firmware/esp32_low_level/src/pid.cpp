#include <algorithm>

#include "pid.hpp"

namespace app {

void PidController::configure(float kp, float ki, float kd, float output_min, float output_max) {
    setGains(kp, ki, kd);
    setOutputClamp(output_min, output_max);
}

void PidController::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PidController::setIntegralClamp(float integral_min, float integral_max) {
    integral_min_ = integral_min;
    integral_max_ = integral_max;
}

void PidController::setOutputClamp(float output_min, float output_max) {
    output_min_ = output_min;
    output_max_ = output_max;
}

void PidController::reset() {
    integral_ = 0.0f;
    previous_error_ = 0.0f;
    has_previous_error_ = false;
}

float PidController::update(float target, float measured, float dt_seconds, bool enable_integral) {
    const float error = target - measured;
    float derivative = 0.0f;
    if (has_previous_error_ && dt_seconds > 0.0f) {
        derivative = (error - previous_error_) / dt_seconds;
    }

    const float proportional = kp_ * error;
    const float derivative_term = kd_ * derivative;

    if (enable_integral && dt_seconds > 0.0f) {
        const float candidate_integral = std::clamp(integral_ + error * dt_seconds, integral_min_, integral_max_);
        const float candidate_output = proportional + derivative_term + ki_ * candidate_integral;
        const bool saturating_high = candidate_output > output_max_ && error > 0.0f;
        const bool saturating_low = candidate_output < output_min_ && error < 0.0f;
        if (!(saturating_high || saturating_low)) {
            integral_ = candidate_integral;
        }
    }

    previous_error_ = error;
    has_previous_error_ = true;

    const float output = proportional + ki_ * integral_ + derivative_term;
    return std::clamp(output, output_min_, output_max_);
}

}  // namespace app
