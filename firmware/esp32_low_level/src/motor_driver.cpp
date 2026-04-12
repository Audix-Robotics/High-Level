#include <Arduino.h>

#include <cmath>

#include "motor_driver.hpp"

namespace app {

namespace {

float applyDeadband(float pwm_value) {
	if (pwm_value == 0.0f) {
		return 0.0f;
	}

	const float sign = (pwm_value >= 0.0f) ? 1.0f : -1.0f;
	const float magnitude = clampFloat(std::fabs(pwm_value), 0.0f, static_cast<float>(PWM_MAX));
	if (magnitude < MOTOR_DEADBAND) {
		return sign * MOTOR_DEADBAND;
	}
	return sign * magnitude;
}

void applySingleMotor(std::size_t wheel_index, float pwm_command) {
	const MotorPinConfig& motor = MOTOR_PINS[wheel_index];
	float signed_pwm = clampFloat(pwm_command, -static_cast<float>(PWM_MAX), static_cast<float>(PWM_MAX));
	signed_pwm *= static_cast<float>(MOTOR_POLARITY[wheel_index]);
	signed_pwm = applyDeadband(signed_pwm);

	const bool forward = signed_pwm >= 0.0f;
	const std::uint32_t duty = static_cast<std::uint32_t>(clampFloat(std::fabs(signed_pwm), 0.0f, static_cast<float>(PWM_MAX)));

	if (forward) {
		ledcWrite(motor.input_a_channel, duty);
		ledcWrite(motor.input_b_channel, 0);
	} else {
		ledcWrite(motor.input_a_channel, 0);
		ledcWrite(motor.input_b_channel, duty);
	}
}

}  // namespace

void initializeMotorDriver() {
	for (std::size_t index = 0; index < WHEEL_COUNT; ++index) {
		const MotorPinConfig& motor = MOTOR_PINS[index];
		pinMode(motor.input_a_pin, OUTPUT);
		pinMode(motor.input_b_pin, OUTPUT);
		digitalWrite(motor.input_a_pin, LOW);
		digitalWrite(motor.input_b_pin, LOW);
		ledcSetup(motor.input_a_channel, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
		ledcSetup(motor.input_b_channel, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
		ledcAttachPin(motor.input_a_pin, motor.input_a_channel);
		ledcAttachPin(motor.input_b_pin, motor.input_b_channel);
		ledcWrite(motor.input_a_channel, 0);
		ledcWrite(motor.input_b_channel, 0);
	}
}

void applyMotorOutputs(const std::array<float, WHEEL_COUNT>& pwm_outputs) {
	for (std::size_t index = 0; index < WHEEL_COUNT; ++index) {
		applySingleMotor(index, pwm_outputs[index]);
	}
}

void stopAllMotors() {
	for (std::size_t index = 0; index < WHEEL_COUNT; ++index) {
		const MotorPinConfig& motor = MOTOR_PINS[index];
		digitalWrite(motor.input_a_pin, LOW);
		digitalWrite(motor.input_b_pin, LOW);
		ledcWrite(motor.input_a_channel, 0);
		ledcWrite(motor.input_b_channel, 0);
	}
}

}  // namespace app
