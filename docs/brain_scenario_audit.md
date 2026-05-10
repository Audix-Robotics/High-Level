# Brain Scenario Audit

## Current Entrypoints

### Arena wrapper tree

- scripts/clean_launch_arena.sh
- src/audix_pkg/launch/scenarios/full_mission.launch.py
- src/audix_pkg/launch/stacks/full_mission_world.launch.py
- src/audix_pkg/launch/stacks/scissor_gazebo.launch.py
- src/audix_pkg/scripts/brains/arena_roamer.py
- src/audix_pkg/scripts/support/arena_ir_state_adapter.py
- src/audix_pkg/scripts/support/mecanum_kinematics.py
- src/audix_pkg/scripts/support/scissor_lift_mapper.py
- src/audix_pkg/scripts/visualization/warehouse_overlay_markers.py
- src/audix_pkg/scripts/support/arena_obstacle_manager.py
- src/audix_pkg/scripts/ui/start_stop_gui.py

### Stress wrapper tree

- scripts/clean_launch_stress_course.sh
- src/audix_pkg/launch/scenarios/arena_stress_course.launch.py
- src/audix_pkg/launch/stacks/arena_stress_world.launch.py
- src/audix_pkg/launch/stacks/scissor_gazebo.launch.py
- src/audix_pkg/scripts/brains/arena_roamer.py
- src/audix_pkg/scripts/support/arena_course_spawner.py
- src/audix_pkg/scripts/support/arena_ir_state_adapter.py
- src/audix_pkg/scripts/support/mecanum_kinematics.py
- src/audix_pkg/scripts/support/scissor_lift_mapper.py
- src/audix_pkg/scripts/ui/start_stop_gui.py

### New isolated simple-cardinal scenario

- scripts/clean_launch_simple_cardinal.sh
- src/audix_pkg/launch/scenarios/arena_simple_cardinal.launch.py
- src/audix_pkg/launch/stacks/arena_stress_world.launch.py
- src/audix_pkg/launch/stacks/scissor_gazebo.launch.py
- src/audix_pkg/scripts/brains/simple_cardinal_brain.py
- src/audix_pkg/scripts/support/arena_course_spawner.py
- src/audix_pkg/scripts/support/arena_ir_state_adapter.py
- src/audix_pkg/scripts/support/mecanum_kinematics.py
- src/audix_pkg/scripts/support/scissor_lift_mapper.py
- src/audix_pkg/scripts/ui/start_stop_gui.py

## Brain Files

### Live brains

- src/audix_pkg/scripts/brains/arena_roamer.py: current large simulation brain.
- src/audix_pkg/scripts/brains/simple_cardinal_brain.py: isolated first-pass scenario brain using only encoder odom, IMU, binary IR, and /robot_enable.

### Legacy or archive candidates

- src/audix_pkg/scripts/legacy/mission_controller.py
- src/audix_pkg/scripts/legacy/warehouse_robot.py
- src/audix_pkg/scripts/legacy/obstacle_avoidance.py
- src/audix_pkg/scripts/legacy/waypoints_control.py
- src/audix_pkg/scripts/legacy/waypoints_final.py

### Shared support

- src/audix_pkg/scripts/support/mecanum_kinematics.py
- src/audix_pkg/scripts/support/arena_ir_state_adapter.py
- src/audix_pkg/scripts/support/ir_digital_bridge.py
- src/audix_pkg/scripts/support/scissor_lift_mapper.py
- src/audix_pkg/scripts/support/arena_course_spawner.py
- src/audix_pkg/scripts/support/arena_obstacle_manager.py
- src/audix_pkg/scripts/visualization/warehouse_overlay_markers.py
- src/audix_pkg/scripts/ui/start_stop_gui.py
- src/audix_pkg/scripts/ui/start_stop_node.py
- src/audix_pkg/scripts/tools/goal_sender_node.py

## Hard Rules For New Brains

- The brain subscribes only to encoder-derived odom, IMU, binary IR state, and /robot_enable.
- The brain never reads Gazebo truth pose, world services, spawned obstacle metadata, or RViz interaction topics.
- The brain can know fixed robot geometry, fixed sensor offsets, fixed safety buffers, and scenario goal definitions.
- The brain publishes only /cmd_vel, scissor lift commands when needed, and debug topics.
- Motion stays cardinal for the first comparison scenarios unless a later scenario explicitly tests rotation-heavy logic.
- All scenario-specific tuning stays in scenario YAML, not inside the brain code.

## Measured Geometry

- Base mesh bounding box in base_link frame: 0.38834 m long, 0.31100 m wide.
- Half-length: 0.19417 m.
- Half-width: 0.15550 m.
- Front IR offset: (-0.20195, 0.00482).
- Front-left IR offset: (-0.18323, -0.11962).
- Front-right IR offset: (-0.17753, 0.12688).
- Left IR offset: (-0.00537, -0.15346).
- Right IR offset: (0.00273, 0.15504).
- Back IR offset: (0.16255, -0.00323).

## Constraint Updates Applied In This Session

- All six simulation IR sensors now use an 8 cm max range.
- The IR adapter trip distances are now 8 cm for all six sensors.
- Default IR debug marker distances are now 8 cm.
- The simulation EKF now uses /mecanum_odom instead of bridged Gazebo /odom.
- The mecanum kinematics node now clamps wheel commands to 60 rpm.
- The spawn panel is disabled by default in the active arena and hardware launches.

## Massive TODO

- Move legacy control scripts into a clear archive folder and stop installing them by default.
- Split arena_roamer.py into support utilities plus scenario brains.
- Add a scenario matrix document listing each launch, brain, world, and success criteria.
- Create a lift-enabled simple warehouse scenario that keeps the isolated brain rule.
- Add a second comparison brain with deterministic left-first then right-switch logic.
- Add a third comparison brain with alternating side preference and loop counters.
- Add a headless smoke runner for each scenario.
- Record expected sensor topics, odom topic, IMU topic, and debug topics per scenario.
- Add RViz presets for simple-cardinal, warehouse-lift, and stress-course comparisons.
- Remove or archive stale README references to midterm.launch.py and arena_experiment.launch.py.
- Decide whether hardware.launch.py should keep arena_obstacle_manager.py at all.
- Decide whether goal_sender_node.py stays or is replaced with scenario-local mission triggers.
- Add a hardware-specific encoder odom source plan for the ESP32 to Pi path.
- Add a UART and pinout adaptation layer document for the Raspberry Pi deployment path.
- Add scenario acceptance checks for: no spawn panel, start/stop present, binary IR only, 60 rpm cap, encoder-only pose use, RViz path markers present.

## End Goal Deliverables

- A small set of launchable scenarios with one isolated brain file each.
- One comparison table covering behavior quality, reroute success rate, and code complexity.
- A final Raspberry Pi candidate brain chosen from those scenario results.