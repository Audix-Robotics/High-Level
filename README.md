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
- `src/audix_pkg/launch/scenarios/full_mission.launch.py`: Active warehouse simulation scenario entrypoint.
- `src/audix_pkg/launch/scenarios/arena_stress_course.launch.py`: Active stress-course scenario entrypoint.
- `src/audix_pkg/launch/scenarios/arena_simple_cardinal.launch.py`: Isolated simple-cardinal comparison scenario with a single obstacle.
- `src/audix_pkg/launch/stacks/scissor_gazebo.launch.py`: Lower-level Gazebo + robot spawn stack used by the scenario launches.
- `src/audix_pkg/urdf/audix.urdf`: Robot description; sensor frame origins are defined here and must match the brain logic.
- `src/audix_pkg/config/common/ekf.yaml`: EKF configuration (frame names, sensor sources, covariances).
- `src/audix_pkg/config/common/mission_params.yaml`: Shared kinematic and mission defaults.
- `src/audix_pkg/config/scenarios/`: Scenario-specific tuning and obstacle course JSON files.

Main code that handles detection and reroute logic
- `src/audix_pkg/scripts/brains/arena_roamer.py`: Primary obstacle detection and avoidance node for the arena experiment. Handles IR topic subscriptions, binary sensor sequencing, mapping sensor names to topics, and the local reroute (3-point) behavior. This file contains the sensor-to-topic remapping and the visual markers used for debugging.
- `src/audix_pkg/scripts/brains/simple_cardinal_brain.py`: Isolated cardinal-motion comparison brain that consumes only encoder odom, IMU, binary IR, and `/robot_enable`.
- `src/audix_pkg/scripts/support/arena_obstacle_manager.py`: Runtime obstacle spawning and tracking for the warehouse arena.
- `src/audix_pkg/scripts/tools/cardinal_motion_debug.py`: Debug utility for cardinal motion tests and sensor offset tuning.
- `src/audix_pkg/scripts/legacy/`: Archived older controllers and waypoint logic kept aside from the active simulation path.

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
- `src/audix_pkg/scripts/brains/arena_roamer.py` — topic remapping, `sensor_positions`, `_sensor_direction_body`, and IR handling code.
- `src/audix_pkg/scripts/brains/simple_cardinal_brain.py` — isolated reroute logic for the simple comparison scenario.
- `src/audix_pkg/config/scenarios/simple_cardinal.yaml` and `src/audix_pkg/config/common/mission_params.yaml` — thresholds that control the active cardinal scenario and shared motion defaults.

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

**TODO (Personal Reminders)**

- Add configurable waypoint stop logic in `mission_controller.py` (parameterize stop/dwell behavior).
- Make obstacle spawn size configurable and use a `default_obstacle_size` parameter in spawner code.
- Change obstacle sizes in mission/config files to reflect real-world tests and tuning.
- Improve the spawner GUI (`arena_spawn_panel.py` / `arena_obstacle_manager.py`) — better size/pose controls and presets.
- Clean up unused scripts and tidy the codebase (remove or archive deprecated tools).
- Update and clean this README with a developer checklist and testing steps.
- Generate a node ↔ topic ↔ file map (which node publishes/subscribes to each topic/service, and what script implements it).
- Run build & syntax checks: `colcon build --symlink-install --packages-select audix` and `python3 -m py_compile <script>`.
- Smoke test in simulation and record real observations (behavior at waypoints, obstacle detection, reroute correctness).

Put these on your short-term backlog and mark them done as you complete them.
