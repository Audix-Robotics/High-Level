# Audix ROS2 Jazzy Workspace

Human-friendly summary of what the repo contains, how to run the full simulation, and where the detection/reroute logic lives.

Top-level idea
- Package: `audix` (source in `src/audix_pkg`)
- Simulation stack: ROS 2 Jazzy + Gazebo (gz-sim) + rviz2 for visualization

Quick run (one command)
Run this from the repository root to launch the arena simulation with sane cleanup and a single RViz window:

```bash
./scripts/clean_launch_arena.sh
```

What `./scripts/clean_launch_arena.sh` does
- Cleans up any stale simulation or visualization processes
- Launches the Gazebo-based arena launch and associated nodes
- Starts RViz after a short delay and tracks its PID for clean shutdown

Key files and their purpose
- `src/audix_pkg/launch/midterm.launch.py`: Main convenience launch used for classroom demos. Starts the Gazebo world and most nodes.
- `src/audix_pkg/launch/scissor_gazebo.launch.py`: Lower-level Gazebo + robot spawn launch (used by `midterm.launch.py`).
- `src/audix_pkg/launch/arena_experiment.launch.py`: Arena experiment launch that starts the Gazebo world, bridges, and experiment nodes. Note: RViz start was removed from this launch (RViz is started by `./scripts/clean_launch_arena.sh` to avoid duplicate windows).
- `src/audix_pkg/urdf/audix.urdf`: Robot description; sensor frame origins are defined here and must match `sensor_positions` in the code.
- `src/audix_pkg/config/ekf.yaml`: EKF configuration (frame names, sensor sources, covariances).
- `src/audix_pkg/config/mission_params.yaml` and `src/audix_pkg/config/arena_experiment_params.yaml`: Tunable experiment and mission parameters (waypoints, thresholds, IR ranges, reroute timings).

Main code that handles detection and reroute logic
- `src/audix_pkg/scripts/arena_roamer.py`: Primary obstacle detection and avoidance node for the arena experiment. Handles IR topic subscriptions, binary sensor sequencing, mapping sensor names to topics, and the local reroute (3-point) behavior. This file contains the sensor-to-topic remapping and the visual markers used for debugging.
- `src/audix_pkg/scripts/mission_controller.py`: Higher-level mission FSM (waypoint sequencing, EKF-based navigation, and mission-level reroute/probe sequencing). Contains mission parameters and the code that triggers reroutes when blocking obstacles are found.
- `src/audix_pkg/scripts/arena_obstacle_manager.py`: Runtime obstacle spawning and tracking (used for experiments and replaying obstacle layouts).
- `src/audix_pkg/scripts/cardinal_motion_debug.py`: Helpful debug utilities for cardinal motion tests and sensor offset tuning.

Useful commands
- Build and source:
```bash
colcon build --symlink-install --packages-select audix
source install/setup.bash
```
- Run the full experiment (recommended):
```bash
./scripts/clean_launch_arena.sh
```
- If you need to run only the launch file (not recommended because it may start a second RViz):
```bash
ros2 launch audix src/audix_pkg/launch/arena_experiment.launch.py
```

Files to inspect when debugging sensors or reroute behavior
- `src/audix_pkg/urdf/audix.urdf` — verify the sensor joint origins and `gazebo` sensor `<range>` settings.
- `src/audix_pkg/scripts/arena_roamer.py` — topic remapping, `sensor_positions`, `_sensor_direction_body`, and IR handling code.
- `src/audix_pkg/scripts/mission_controller.py` — mission-level reroute logic and probe sequencing.
- `src/audix_pkg/config/arena_experiment_params.yaml` and `mission_params.yaml` — thresholds that control when detection → reroute occurs.

Quick checks while sim is running
- Verify sensor topics:
```bash
ros2 topic echo /ir_front/scan --once
ros2 topic echo /ir_left/scan --once
```
- Check odometry and IMU:
```bash
ros2 topic echo /odometry/filtered --once
ros2 topic echo /imu --once
```

Notes
- RViz is intentionally started by `./scripts/clean_launch_arena.sh` to ensure only one RViz window opens and that it is cleaned up properly on exit.
- Keep `use_sim_time: True` for any node running in simulation.
- Do not modify files under `src/audix_pkg/meshes/`.

If you want, I can run a quick TF/`robot_state_publisher` smoke test now to verify the URDF and sensor marker frames. 
