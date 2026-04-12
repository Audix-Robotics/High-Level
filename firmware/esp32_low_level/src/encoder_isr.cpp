#include <Arduino.h>

#include "encoder_isr.hpp"
#include "shared_state.hpp"

namespace app {

namespace {

volatile std::uint8_t g_last_state[WHEEL_COUNT] = {0, 0, 0, 0};
constexpr std::int8_t QUADRATURE_TABLE[16] = {
	0, -1, 1, 0,
	1, 0, 0, -1,
	-1, 0, 0, 1,
	0, 1, -1, 0,
};

inline std::uint8_t readWheelState(WheelIndex wheel) {
	const EncoderPinConfig& pins = ENCODER_PINS[wheel];
	const std::uint8_t a = static_cast<std::uint8_t>(digitalRead(pins.channel_a_pin));
	const std::uint8_t b = static_cast<std::uint8_t>(digitalRead(pins.channel_b_pin));
	return static_cast<std::uint8_t>((a << 1) | b);
}

inline void IRAM_ATTR handleEncoderEdge(WheelIndex wheel) {
	const std::uint8_t current_state = readWheelState(wheel);
	const std::uint8_t previous_state = g_last_state[wheel];
	const std::uint8_t transition = static_cast<std::uint8_t>((previous_state << 2) | current_state);
	const std::int8_t delta = QUADRATURE_TABLE[transition] * ENCODER_POLARITY[wheel];
	if (delta != 0) {
		sharedState().addEncoderDeltaFromIsr(wheel, delta);
	}
	g_last_state[wheel] = current_state;
}

}  // namespace

void initializeEncoders() {
	for (std::size_t index = 0; index < WHEEL_COUNT; ++index) {
		const EncoderPinConfig& pins = ENCODER_PINS[index];
		pinMode(pins.channel_a_pin, INPUT_PULLUP);
		pinMode(pins.channel_b_pin, INPUT_PULLUP);
		g_last_state[index] = readWheelState(static_cast<WheelIndex>(index));
	}

	attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[FL].channel_a_pin), onFrontLeftEncoderEdge, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[FL].channel_b_pin), onFrontLeftEncoderEdge, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[FR].channel_a_pin), onFrontRightEncoderEdge, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[FR].channel_b_pin), onFrontRightEncoderEdge, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[RL].channel_a_pin), onRearLeftEncoderEdge, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[RL].channel_b_pin), onRearLeftEncoderEdge, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[RR].channel_a_pin), onRearRightEncoderEdge, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[RR].channel_b_pin), onRearRightEncoderEdge, CHANGE);
}

std::array<std::int32_t, WHEEL_COUNT> readEncoderCounts() {
	return sharedState().getEncoderCounts();
}

void resetEncoderCounts() {
	sharedState().resetEncoderCounts();
	for (std::size_t index = 0; index < WHEEL_COUNT; ++index) {
		g_last_state[index] = readWheelState(static_cast<WheelIndex>(index));
	}
}

void IRAM_ATTR onFrontLeftEncoderEdge() {
	handleEncoderEdge(FL);
}

void IRAM_ATTR onFrontRightEncoderEdge() {
	handleEncoderEdge(FR);
}

void IRAM_ATTR onRearLeftEncoderEdge() {
	handleEncoderEdge(RL);
}

void IRAM_ATTR onRearRightEncoderEdge() {
	handleEncoderEdge(RR);
}

}  // namespace app
