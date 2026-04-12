#include <Arduino.h>
#include <Wire.h>

#include "config.hpp"
#include "imu_driver.hpp"

namespace app {

namespace {

constexpr std::uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr std::uint8_t REG_CONFIG = 0x1A;
constexpr std::uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr std::uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr std::uint8_t REG_ACCEL_XOUT_H = 0x3B;

constexpr float ACCEL_SCALE_M_S2 = GRAVITY_M_S2 / 16384.0f;
constexpr float GYRO_SCALE_RAD_S = (PI_F / 180.0f) / 131.0f;

bool g_imu_ok = false;
float g_yaw_estimate = 0.0f;

bool writeRegister(std::uint8_t reg, std::uint8_t value) {
	Wire.beginTransmission(IMU_I2C_ADDRESS);
	Wire.write(reg);
	Wire.write(value);
	return Wire.endTransmission() == 0;
}

bool readBytes(std::uint8_t reg, std::uint8_t* buffer, std::size_t length) {
	Wire.beginTransmission(IMU_I2C_ADDRESS);
	Wire.write(reg);
	if (Wire.endTransmission(false) != 0) {
		return false;
	}

	const std::size_t received = Wire.requestFrom(static_cast<int>(IMU_I2C_ADDRESS), static_cast<int>(length), static_cast<int>(true));
	if (received != length) {
		return false;
	}

	for (std::size_t index = 0; index < length; ++index) {
		buffer[index] = static_cast<std::uint8_t>(Wire.read());
	}
	return true;
}

std::int16_t combineBytes(std::uint8_t msb, std::uint8_t lsb) {
	return static_cast<std::int16_t>((static_cast<std::uint16_t>(msb) << 8) | lsb);
}

}  // namespace

bool initializeImu() {
	Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_FREQUENCY);
	delay(50);

	const bool ok =
		writeRegister(REG_PWR_MGMT_1, 0x00) &&
		writeRegister(REG_CONFIG, 0x03) &&
		writeRegister(REG_GYRO_CONFIG, 0x00) &&
		writeRegister(REG_ACCEL_CONFIG, 0x00);

	g_imu_ok = ok;
	g_yaw_estimate = 0.0f;
	return g_imu_ok;
}

bool readImu(IMUState& state, float dt_seconds) {
	std::uint8_t raw_data[14] = {0};
	if (!g_imu_ok || !readBytes(REG_ACCEL_XOUT_H, raw_data, sizeof(raw_data))) {
		g_imu_ok = false;
		return false;
	}

	const std::int16_t ax_raw = combineBytes(raw_data[0], raw_data[1]);
	const std::int16_t ay_raw = combineBytes(raw_data[2], raw_data[3]);
	const std::int16_t az_raw = combineBytes(raw_data[4], raw_data[5]);
	const std::int16_t gx_raw = combineBytes(raw_data[8], raw_data[9]);
	const std::int16_t gy_raw = combineBytes(raw_data[10], raw_data[11]);
	const std::int16_t gz_raw = combineBytes(raw_data[12], raw_data[13]);

	state.accel_x = static_cast<float>(ax_raw) * ACCEL_SCALE_M_S2;
	state.accel_y = static_cast<float>(ay_raw) * ACCEL_SCALE_M_S2;
	state.accel_z = static_cast<float>(az_raw) * ACCEL_SCALE_M_S2;
	state.gyro_x = static_cast<float>(gx_raw) * GYRO_SCALE_RAD_S;
	state.gyro_y = static_cast<float>(gy_raw) * GYRO_SCALE_RAD_S;
	state.gyro_z = static_cast<float>(gz_raw) * GYRO_SCALE_RAD_S;

	g_yaw_estimate = wrapAngle(g_yaw_estimate + state.gyro_z * dt_seconds);
	state.orientation_z = g_yaw_estimate;
	return true;
}

bool imuHealthy() {
	return g_imu_ok;
}

}  // namespace app
