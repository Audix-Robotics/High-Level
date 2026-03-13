# Audix ROS2 Simulation Workspace

## Project Status (Where We Reached)
This workspace has been migrated to ROS 2 (Jazzy) with Gazebo (ros_gz) and is currently running a working simulation pipeline for the Audix robot.

Implemented so far:
- ROS 2 package setup with `ament_cmake` and install rules.
- Gazebo simulation launch with world loading and robot spawning.
- URDF integration with `robot_state_publisher`.
- `ros_gz_bridge` topic bridges for clock, odometry, joint states, IMU, lidar scan, and velocity command.
- `ros2_control` + `diff_drive_controller` integration.
- EKF localization node (`robot_localization`) enabled for fused state estimation.
- RViz launch and default visualization config.
- Initial behavior scripts for obstacle avoidance and waypoint motion.

Current known scope:
- Main drivetrain currently behaves as differential drive in simulation.
- Team roadmap below includes planned migration/tuning toward mecanum-specific behavior and a fuller warehouse-like environment.

## Quick Start
Use these commands from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select audix
source install/setup.bash
```

Launch Gazebo simulation:

```bash
ros2 launch audix robot_gazebo.launch.py
```

Launch RViz visualization-only flow:

```bash
ros2 launch audix display_rviz.launch.py
```

## What Each Launch File Does
- `src/audix_pkg/launch/robot_gazebo.launch.py`
  - Main simulation entrypoint.
  - Starts Gazebo, robot state publisher, bridges, robot spawn, and EKF.
- `src/audix_pkg/launch/gazebo.launch.py`
  - Wrapper that includes `robot_gazebo.launch.py`.
- `src/audix_pkg/launch/display_rviz.launch.py`
  - RViz-focused launch (URDF, joint state publisher/gui, robot state publisher, RViz2).
- `src/audix_pkg/launch/display.launch.py`
  - Wrapper that includes `display_rviz.launch.py`.

## Config and World Files (High-Level)
- `src/audix_pkg/config/controllers.yaml`
  - Controller manager and drive controller parameters (wheel names, geometry, limits, cmd format).
- `src/audix_pkg/config/ekf.yaml`
  - EKF fusion settings (odom + IMU) and TF publishing behavior.
- `src/audix_pkg/rviz/config.rviz`
  - Default RViz display layout and fixed frame.
- `src/audix_pkg/world/my_world.sdf`
  - Main simulation world (ground plane, lighting, example obstacle, physics/sensor systems).
- `src/audix_pkg/world/empty_world.sdf`
  - Minimal baseline world variant.

## Simulation Pipeline Overview
1. Build and source the workspace.
2. Launch Gazebo world.
3. Publish robot URDF to `robot_description`.
4. Spawn robot entity in Gazebo from `robot_description`.
5. Bridge Gazebo/ROS topics (`/clock`, `/odom`, `/imu`, `/scan`, `/joint_states`, `/cmd_vel`).
6. Run control and localization (`ros2_control`, diff drive, EKF).
7. Optionally visualize in RViz.

## Team Roadmap (Organized TODO)

### A. Robot Frames, Geometry, and Spawn Accuracy
1. Make `RobotBody` coincide with the robot CG (not a corner/reference artifact).
2. Update spawn height so robot starts above the floor by the CG-to-lowest-wheel-point offset.
3. Add dedicated IR sensor frames in SolidWorks and keep URDF frame mapping consistent.
4. Add a frame at the exact IMU location for accurate simulation alignment (IMU frame can differ from CG).
5. Simplify RViz defaults by hiding extra mechanism links that reduce readability.

### B. Mecanum Drivetrain Study and Motion Mapping
1. Study mecanum wheel kinematics and control requirements.
2. Define and validate motion mapping for: forward, backward, left, right, rotate, and combined steering.
3. Ensure controller architecture supports mecanum behavior instead of diff-drive assumptions where needed.

### C. Environment and Obstacles
1. Define dynamic obstacle strategy:
   - How many obstacles.
   - Spawning method.
   - Motion behavior and test scenarios.
2. Design shelves and inventory layout to match the expected real environment.
3. Build the full Gazebo environment to reflect expected operational conditions.

### D. Controls, Scripts, and Mechanism Automation
1. Decide what extra scripts are required and how customizable the GUI/control interface should be.
2. Integrate scissor-lift motion mapping (from MATLAB) into an automation script:
   - Camera height command in.
   - Coordinated link motion out.

### E. Midterm Planning and Validation Scope
1. Prepare a clear checklist of all midterm deliverables and acceptance criteria.
2. Decide perception scope:
   - Whether camera sensing/object detection is required in simulation.
   - Required reactions/behaviors if detection is enabled.

### F. Data, Calibration, and Visual Fidelity
1. Collect and track all key dimensions, speeds, and relevant physical parameters to improve simulation accuracy.
2. Apply robot coloring/material styling for clearer visualization and team demos.

## Editing Guidance (For Future Team Members)
- Adjust drivetrain and controller behavior in `config/controllers.yaml` and matching wheel/joint definitions in URDF.
- Update localization behavior in `config/ekf.yaml`.
- Modify world geometry and obstacles in `world/my_world.sdf`.
- Keep frame naming consistent across SolidWorks export, URDF, controllers, and RViz.
- After any change:
  1. Rebuild package.
  2. Re-source `install/setup.bash`.
  3. Relaunch and validate topics/TF.
