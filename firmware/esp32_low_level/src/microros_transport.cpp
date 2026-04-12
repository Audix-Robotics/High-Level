#include <Arduino.h>
#include <math.h>
#include <micro_ros_arduino.h>

#include <builtin_interfaces/msg/time.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>

#include "config.hpp"
#include "microros_transport.hpp"
#include "shared_state.hpp"

namespace app {

namespace {

constexpr std::uint32_t RECONNECT_INTERVAL_MS = 1000;
constexpr std::uint32_t AGENT_PING_INTERVAL_MS = 250;

struct MicroRosContext {
	bool transport_initialized = false;
	bool entities_created = false;
	bool messages_initialized = false;
	std::uint32_t last_reconnect_attempt_ms = 0;
	std::uint32_t last_ping_ms = 0;

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support{};
	rcl_node_t node = rcl_get_zero_initialized_node();
	rcl_subscription_t cmd_vel_sub = rcl_get_zero_initialized_subscription();
	rcl_subscription_t robot_enable_sub = rcl_get_zero_initialized_subscription();
	rcl_publisher_t odom_pub = rcl_get_zero_initialized_publisher();
	rcl_publisher_t imu_pub = rcl_get_zero_initialized_publisher();
	rcl_publisher_t limit_switch_pub = rcl_get_zero_initialized_publisher();
	rclc_executor_t executor{};

	geometry_msgs__msg__Twist cmd_vel_msg{};
	std_msgs__msg__Bool robot_enable_msg{};
	nav_msgs__msg__Odometry odom_msg{};
	sensor_msgs__msg__Imu imu_msg{};
	std_msgs__msg__Bool limit_switch_msg{};
};

MicroRosContext g_microros;

builtin_interfaces__msg__Time toRosTime(std::uint32_t now_ms) {
	builtin_interfaces__msg__Time stamp{};
	stamp.sec = static_cast<int32_t>(now_ms / 1000U);
	stamp.nanosec = static_cast<uint32_t>((now_ms % 1000U) * 1000000U);
	return stamp;
}

void setCovariance36(double* covariance, double x, double y, double z, double roll, double pitch, double yaw) {
	for (std::size_t index = 0; index < 36; ++index) {
		covariance[index] = 0.0;
	}
	covariance[0] = x;
	covariance[7] = y;
	covariance[14] = z;
	covariance[21] = roll;
	covariance[28] = pitch;
	covariance[35] = yaw;
}

void setCovariance9(double* covariance, double x, double y, double z) {
	for (std::size_t index = 0; index < 9; ++index) {
		covariance[index] = 0.0;
	}
	covariance[0] = x;
	covariance[4] = y;
	covariance[8] = z;
}

void fillYawQuaternion(float yaw, geometry_msgs__msg__Quaternion& quaternion) {
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sinf(0.5f * yaw);
	quaternion.w = cosf(0.5f * yaw);
}

void cmdVelCallback(const void* message) {
	const auto* msg = static_cast<const geometry_msgs__msg__Twist*>(message);
	sharedState().setCommand(
		static_cast<float>(msg->linear.x),
		static_cast<float>(msg->linear.y),
		static_cast<float>(msg->angular.z),
		millis());
}

void robotEnableCallback(const void* message) {
	const auto* msg = static_cast<const std_msgs__msg__Bool*>(message);
	sharedState().setRobotEnabled(msg->data);
}

void initializeMessages() {
	if (g_microros.messages_initialized) {
		return;
	}

	nav_msgs__msg__Odometry__init(&g_microros.odom_msg);
	sensor_msgs__msg__Imu__init(&g_microros.imu_msg);
	std_msgs__msg__Bool__init(&g_microros.limit_switch_msg);
	geometry_msgs__msg__Twist__init(&g_microros.cmd_vel_msg);
	std_msgs__msg__Bool__init(&g_microros.robot_enable_msg);

	rosidl_runtime_c__String__assign(&g_microros.odom_msg.header.frame_id, "odom");
	rosidl_runtime_c__String__assign(&g_microros.odom_msg.child_frame_id, "base_link");
	rosidl_runtime_c__String__assign(&g_microros.imu_msg.header.frame_id, "imu_link");

	setCovariance36(g_microros.odom_msg.pose.covariance, 0.02, 0.02, 1e6, 1e6, 1e6, 0.05);
	setCovariance36(g_microros.odom_msg.twist.covariance, 0.02, 0.02, 1e6, 1e6, 1e6, 0.05);
	setCovariance9(g_microros.imu_msg.orientation_covariance, 1e6, 1e6, 0.05);
	setCovariance9(g_microros.imu_msg.angular_velocity_covariance, 0.02, 0.02, 0.02);
	setCovariance9(g_microros.imu_msg.linear_acceleration_covariance, 0.04, 0.04, 0.04);
	g_microros.messages_initialized = true;
}

void destroyEntities() {
	if (!g_microros.entities_created) {
		return;
	}

	rcl_publisher_fini(&g_microros.limit_switch_pub, &g_microros.node);
	rcl_publisher_fini(&g_microros.imu_pub, &g_microros.node);
	rcl_publisher_fini(&g_microros.odom_pub, &g_microros.node);
	rcl_subscription_fini(&g_microros.robot_enable_sub, &g_microros.node);
	rcl_subscription_fini(&g_microros.cmd_vel_sub, &g_microros.node);
	rclc_executor_fini(&g_microros.executor);
	rcl_node_fini(&g_microros.node);
	rclc_support_fini(&g_microros.support);

	g_microros.node = rcl_get_zero_initialized_node();
	g_microros.cmd_vel_sub = rcl_get_zero_initialized_subscription();
	g_microros.robot_enable_sub = rcl_get_zero_initialized_subscription();
	g_microros.odom_pub = rcl_get_zero_initialized_publisher();
	g_microros.imu_pub = rcl_get_zero_initialized_publisher();
	g_microros.limit_switch_pub = rcl_get_zero_initialized_publisher();
	g_microros.support = rclc_support_t{};
	g_microros.executor = rclc_executor_t{};
	g_microros.entities_created = false;
}

bool createEntities() {
	initializeMessages();
	destroyEntities();

	g_microros.allocator = rcl_get_default_allocator();
	g_microros.support = rclc_support_t{};
	g_microros.node = rcl_get_zero_initialized_node();
	g_microros.cmd_vel_sub = rcl_get_zero_initialized_subscription();
	g_microros.robot_enable_sub = rcl_get_zero_initialized_subscription();
	g_microros.odom_pub = rcl_get_zero_initialized_publisher();
	g_microros.imu_pub = rcl_get_zero_initialized_publisher();
	g_microros.limit_switch_pub = rcl_get_zero_initialized_publisher();
	g_microros.executor = rclc_executor_t{};
	g_microros.entities_created = true;

	if (rclc_support_init(&g_microros.support, 0, nullptr, &g_microros.allocator) != RCL_RET_OK) {
		destroyEntities();
		return false;
	}

	if (rclc_node_init_default(&g_microros.node, "audix_esp32_node", "", &g_microros.support) != RCL_RET_OK) {
		destroyEntities();
		return false;
	}

	if (rclc_subscription_init_default(
			&g_microros.cmd_vel_sub,
			&g_microros.node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
			"/cmd_vel") != RCL_RET_OK) {
		destroyEntities();
		return false;
	}

	if (rclc_subscription_init_default(
			&g_microros.robot_enable_sub,
			&g_microros.node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
			"/robot_enable") != RCL_RET_OK) {
		destroyEntities();
		return false;
	}

	if (rclc_publisher_init_default(
			&g_microros.odom_pub,
			&g_microros.node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
			"/odom") != RCL_RET_OK) {
		destroyEntities();
		return false;
	}

	if (rclc_publisher_init_default(
			&g_microros.imu_pub,
			&g_microros.node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
			"/imu") != RCL_RET_OK) {
		destroyEntities();
		return false;
	}

	if (rclc_publisher_init_default(
			&g_microros.limit_switch_pub,
			&g_microros.node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
			"/limit_switch") != RCL_RET_OK) {
		destroyEntities();
		return false;
	}

	if (rclc_executor_init(&g_microros.executor, &g_microros.support.context, 2, &g_microros.allocator) != RCL_RET_OK) {
		destroyEntities();
		return false;
	}

	if (rclc_executor_add_subscription(&g_microros.executor, &g_microros.cmd_vel_sub, &g_microros.cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK) {
		destroyEntities();
		return false;
	}

	if (rclc_executor_add_subscription(&g_microros.executor, &g_microros.robot_enable_sub, &g_microros.robot_enable_msg, &robotEnableCallback, ON_NEW_DATA) != RCL_RET_OK) {
		destroyEntities();
		return false;
	}

	g_microros.last_ping_ms = millis();
	return true;
}

}  // namespace

bool initializeMicroRosTransport() {
	if (!g_microros.transport_initialized) {
		set_microros_serial_transports(Serial);
		g_microros.transport_initialized = true;
		g_microros.last_reconnect_attempt_ms = 0;
		initializeMessages();
	}
	return true;
}

void shutdownMicroRosTransport() {
	destroyEntities();
}

void microrosSpinSome(std::uint32_t timeout_ms) {
	initializeMicroRosTransport();
	const std::uint32_t now_ms = millis();

	if (!g_microros.entities_created) {
		if ((now_ms - g_microros.last_reconnect_attempt_ms) >= RECONNECT_INTERVAL_MS) {
			g_microros.last_reconnect_attempt_ms = now_ms;
			if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
				createEntities();
			}
		}
		return;
	}

	if ((now_ms - g_microros.last_ping_ms) >= AGENT_PING_INTERVAL_MS) {
		g_microros.last_ping_ms = now_ms;
		if (rmw_uros_ping_agent(20, 1) != RMW_RET_OK) {
			destroyEntities();
			return;
		}
	}

	const uint64_t timeout_ns = static_cast<uint64_t>(timeout_ms) * 1000000ULL;
	if (rclc_executor_spin_some(&g_microros.executor, timeout_ns) != RCL_RET_OK) {
		destroyEntities();
	}
}

bool publishTelemetry() {
	if (!g_microros.entities_created) {
		return false;
	}

	const std::uint32_t now_ms = millis();
	const builtin_interfaces__msg__Time stamp = toRosTime(now_ms);
	const OdometryState odom_state = sharedState().getOdometryState();
	const IMUState imu_state = sharedState().getImuState();
	const SensorState sensor_state = sharedState().getSensorState();

	g_microros.odom_msg.header.stamp = stamp;
	g_microros.odom_msg.pose.pose.position.x = odom_state.x;
	g_microros.odom_msg.pose.pose.position.y = odom_state.y;
	g_microros.odom_msg.pose.pose.position.z = 0.0;
	fillYawQuaternion(odom_state.theta, g_microros.odom_msg.pose.pose.orientation);
	g_microros.odom_msg.twist.twist.linear.x = odom_state.vx;
	g_microros.odom_msg.twist.twist.linear.y = odom_state.vy;
	g_microros.odom_msg.twist.twist.angular.z = odom_state.wtheta;

	g_microros.imu_msg.header.stamp = stamp;
	fillYawQuaternion(imu_state.orientation_z, g_microros.imu_msg.orientation);
	g_microros.imu_msg.angular_velocity.x = imu_state.gyro_x;
	g_microros.imu_msg.angular_velocity.y = imu_state.gyro_y;
	g_microros.imu_msg.angular_velocity.z = imu_state.gyro_z;
	g_microros.imu_msg.linear_acceleration.x = imu_state.accel_x;
	g_microros.imu_msg.linear_acceleration.y = imu_state.accel_y;
	g_microros.imu_msg.linear_acceleration.z = imu_state.accel_z;

	g_microros.limit_switch_msg.data = sensor_state.limit_switch_pressed;

	if (rcl_publish(&g_microros.odom_pub, &g_microros.odom_msg, nullptr) != RCL_RET_OK ||
		rcl_publish(&g_microros.imu_pub, &g_microros.imu_msg, nullptr) != RCL_RET_OK ||
		rcl_publish(&g_microros.limit_switch_pub, &g_microros.limit_switch_msg, nullptr) != RCL_RET_OK) {
		destroyEntities();
		return false;
	}

	return true;
}

bool microrosConnected() {
	return g_microros.entities_created;
}

}  // namespace app
