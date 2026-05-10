cmake_minimum_required(VERSION 3.8)

project(audix)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
	DIRECTORY config launch meshes urdf rviz world models scripts
	DESTINATION share/${PROJECT_NAME}
)

install(
	PROGRAMS
		scripts/brains/arena_roamer.py
		scripts/brains/simple_cardinal_brain.py
		scripts/support/arena_course_spawner.py
		scripts/support/arena_ir_state_adapter.py
		scripts/support/arena_route_library.py
		scripts/support/arena_obstacle_manager.py
		scripts/ui/arena_spawn_panel.py
		scripts/tools/cardinal_motion_debug.py
		scripts/tools/drive_test.py
		scripts/tools/goal_sender_node.py
		scripts/support/mecanum_kinematics.py
		scripts/support/odom_tf_broadcaster.py
		scripts/support/scissor_lift_mapper.py
		scripts/ui/scissor_slider_gui.py
		scripts/ui/start_stop_node.py
		scripts/ui/start_stop_gui.py
		scripts/support/ir_digital_bridge.py
		scripts/visualization/warehouse_overlay_markers.py
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
