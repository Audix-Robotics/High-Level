cmake_minimum_required(VERSION 3.8)

project(audix)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/RobotStatus.msg"
	"msg/FleetStatus.msg"
	"srv/SpawnRobot.srv"
	"srv/AssignMission.srv"
	"srv/GetRobotStatus.srv"
	"srv/DespawnRobot.srv"
	DEPENDENCIES geometry_msgs
)

install(
	DIRECTORY config launch meshes urdf rviz world models scripts
	DESTINATION share/${PROJECT_NAME}
)

install(
	PROGRAMS
		scripts/arena_obstacle_manager.py
		scripts/arena_roamer.py
		scripts/arena_spawn_panel.py
		scripts/arena_spawn_panel_multibot.py
		scripts/cardinal_motion_debug.py
		scripts/drive_test.py
		scripts/ekf_input_adapter.py
		scripts/fleet_robot_navigator.py
		scripts/goal_sender_node.py
		scripts/mecanum_kinematics.py
		scripts/mission_controller.py
		scripts/obstacle_avoidance.py
		scripts/warehouse_fleet_manager.py
		scripts/warehouse_mission_manager.py
		scripts/warehouse_robot.py
		scripts/warehouse_waypoint_editor.py
		scripts/odom_tf_broadcaster.py
		scripts/scissor_lift_mapper.py
		scripts/scissor_slider_gui.py
		scripts/start_stop_node.py
		scripts/start_stop_gui.py
		scripts/waypoints_control.py
		scripts/waypoints_final.py
		scripts/ir_digital_bridge.py
	DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
