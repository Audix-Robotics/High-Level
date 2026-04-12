# Audix Workspace

This workspace is organized around one protected high-level behavior and three explicit execution paths:

- Raspberry Pi real hardware: `pi_hardware.launch.py`
- Raspberry Pi mock validation: `pi_hardware_mock.launch.py`
- Gazebo simulation: `full_mission.launch.py` with `scissor_gazebo.launch.py`

The protected behavior is `arena_roamer.py`. Its navigation, obstacle detection, reroute, and motion-selection logic should be preserved. The rest of the system exists to feed it the right contracts.

## Architecture

Pi responsibilities:

- Run `arena_roamer.py`
- Run `robot_localization` EKF
- Publish `/odometry/filtered`
- Bridge or adapt low-level sensor inputs into the topics expected by `arena_roamer.py`
- Publish `/robot_enable` when the mission should run

ESP32 responsibilities:

- Subscribe to `/cmd_vel`
- Subscribe to `/robot_enable`
- Publish raw `/odom`
- Publish raw `/imu`
- Publish `/limit_switch`
- Handle encoders, IMU, motor output, mecanum inverse kinematics, PID, and local safety timeout

The ESP32 does not own EKF, obstacle logic, or mission logic.

## Main Entrypoints

Real Raspberry Pi hardware:

```bash
colcon build --symlink-install --packages-select audix
source install/setup.bash
ros2 launch audix pi_hardware.launch.py
```

Useful launch arguments:

```bash
ros2 launch audix pi_hardware.launch.py use_micro_ros_agent:=true serial_device:=/dev/ttyUSB0 serial_baud:=115200 use_start_stop:=true auto_start:=true
```

Mock validation path:

```bash
source install/setup.bash
ros2 launch audix pi_hardware_mock.launch.py ir_scenario:=reroute_left_case
```

Gazebo simulation path:

```bash
source install/setup.bash
ros2 launch audix full_mission.launch.py
```

## Source Of Truth Files

Core high-level behavior:

- `src/audix_pkg/scripts/arena_roamer.py`
- `docs/arena_roamer_contract.md`
- `docs/arena_roamer_behavior_baseline.md`

Pi-side launch and integration:

- `src/audix_pkg/launch/pi_hardware.launch.py`
- `src/audix_pkg/launch/pi_hardware_mock.launch.py`
- `src/audix_pkg/scripts/ir_digital_bridge.py`
- `src/audix_pkg/config/hardware/ekf.yaml`

Simulation:

- `src/audix_pkg/launch/full_mission.launch.py`
- `src/audix_pkg/launch/scissor_gazebo.launch.py`

Contracts and architecture docs:

- `docs/system_architecture.md`
- `interface/message_contract.md`
- `interface/pi_esp_topics.md`
- `docs/mock_validation_scenarios.md`

ESP32 firmware:

- `firmware/esp32_low_level/`

## Mock Validation

The mock path is intended for validating `arena_roamer.py` without hardware changes.

Examples:

```bash
ros2 launch audix pi_hardware_mock.launch.py ir_scenario:=all_clear
ros2 launch audix pi_hardware_mock.launch.py ir_scenario:=front_blocked
ros2 launch audix pi_hardware_mock.launch.py ir_scenario:=reroute_left_case
ros2 launch audix pi_hardware_mock.launch.py ir_scenario:=obstacle_reappears_during_rejoin
```

Optional monitor:

```bash
python3 tools/mock_hardware/mock_validation_monitor.py
```

See `tools/mock_hardware/README.md` and `docs/mock_validation_scenarios.md` for scenario expectations and observation guidance.

## Legacy And Reference Files

These files are kept for reference and should not be treated as the current main architecture path:

- `src/audix_pkg/launch/hardware.launch.py`
- `src/audix_pkg/scripts/mecanum_kinematics.py`
- `src/audix_pkg/scripts/mission_controller.py`

They reflect earlier integration stages and are not the recommended entrypoints for the current Pi-high-level / ESP-low-level split.

## Verification

Build the ROS package:

```bash
colcon build --symlink-install --packages-select audix
```

Check the main runtime topics:

```bash
ros2 topic echo /odometry/filtered --once
ros2 topic echo /imu --once
ros2 topic echo /odom --once
ros2 topic echo /robot_enable --once
ros2 topic echo /ir_front/scan --once
```

For hardware and mock path validation, the expectation is:

- low-level layer produces `/odom`, `/imu`, and sensor inputs
- Pi EKF produces `/odometry/filtered`
- `arena_roamer.py` consumes filtered odometry and scan topics, then publishes `/cmd_vel`

## Constraints

- Do not redesign `arena_roamer.py` behavior unless explicitly requested
- Do not move EKF onto the ESP32
- Do not hardcode mission waypoints into Python when they belong in config
- Do not modify `meshes/`
