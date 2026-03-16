# Audix — Warehouse Inventory Robot

## What this is
ROS2 Jazzy + Gazebo Harmonic simulation of a 4-wheel mecanum drive robot with a 3-stage scissor lift and 6 IR obstacle-avoidance sensors. This is a midterm project for MCT334 at Ain Shams University, worth 12 marks.

## Stack
- ROS2 Jazzy on Ubuntu 24.04
- Gazebo Harmonic (gz-sim)
- gz_ros2_control for joint control
- robot_localization (EKF) for sensor fusion
- Python 3 for all nodes

## Package structure
```
src/audix/
├── config/
│   ├── controllers.yaml      # mecanum_velocity_controller + scissor_position_controller
│   ├── ekf.yaml              # robot_localization EKF config
│   └── mission_params.yaml   # ALL tunable params: waypoints, thresholds, lift heights
├── launch/
│   └── midterm.launch.py     # Master launch — everything starts from here
├── scripts/
│   ├── mission_controller.py # Brain: 10-state machine, obstacle avoidance, lift sequencing
│   ├── mecanum_kinematics.py # Twist -> 4 wheel velocities + forward kinematics odometry
│   ├── start_stop_node.py    # Publishes /robot_enable Bool
│   └── goal_sender_node.py   # Calls /send_mission service to trigger mission
├── urdf/audix.urdf           # Full robot description with sensors and ros2_control
├── world/warehouse.sdf       # Warehouse environment with shelves and walls
└── meshes/                   # STL files from SolidWorks (do NOT modify)
```

## Build and run
```bash
cd ~/design_ws
colcon build --symlink-install --packages-select audix
source install/setup.bash
ros2 launch audix midterm.launch.py
```

## Key architecture decisions
- ONE node (mission_controller.py) publishes to /cmd_vel — no topic conflicts
- 6 IR sensors modeled as individual 35° gpu_lidar in Gazebo, each on its own /ir_*/scan topic
- Mecanum kinematics done in a separate node that converts Twist to 4 wheel velocities via JointGroupVelocityController (NOT diff_drive_controller)
- Scissor lift: bottom_stud_joint is the actuated prismatic; scissor_joint_sync computes all 11 passive joint angles from the carriage position
- EKF fuses /mecanum_odom + /imu → publishes /odometry/filtered

## IMPORTANT rules
- NEVER modify files in meshes/ — these are exported from SolidWorks
- NEVER hardcode waypoints in Python — they come from mission_params.yaml
- ALWAYS use `use_sim_time: True` for any node running in simulation
- The world name in the SDF is "warehouse" — bridge remappings must match this
- All Python scripts must have `#!/usr/bin/env python3` shebang and be chmod +x
- When editing the URDF, preserve the exact joint origins from SolidWorks export

## Robot frame convention
- base_link origin is at the SolidWorks assembly origin
- RobotBody is a dummy link fixed to base_link (used as base_frame_id)
- Robot's forward direction is -X in the SolidWorks frame
- Wheel radius: 0.0485m, track half-width: 0.1574m, wheelbase half: 0.09m

## Scissor lift kinematics
- Link length L = 0.120m, 3 stages
- θ_min = 10° (compressed), θ_max = 70° (extended)
- H = 3 * L * sin(θ), carriage_x = L * (cos(θ_min) - cos(θ))
- Carriage range: 0.0 to 0.0772m
- Lead screw: TR8x8, 8mm/rev, NEMA17 stepper 200 steps/rev

## Testing individual components
```bash
# Manual start signal:
ros2 topic pub /robot_enable std_msgs/msg/Bool "{data: true}" --once

# Emergency stop:
ros2 topic pub /robot_enable std_msgs/msg/Bool "{data: false}" --once

# Check IR sensor readings:
ros2 topic echo /ir_front/scan --once

# Check odometry:
ros2 topic echo /odometry/filtered --once

# Manual drive command (strafe right at 0.1 m/s):
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 5
```
