#!/usr/bin/env bash
set -euo pipefail

# Headless launch script for Audix simulation (no GUI components)
# Usage: ./scripts/clean_launch_headless.sh

BASE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$BASE_DIR"

echo "Killing lingering simulator/ROS processes before launch..."
pkill -f "gz sim" || true
pkill -f rviz2 || true
pkill -f gzserver || true
pkill -f gz || true
pkill -f gz_sim || true
pkill -f gzclient || true
pkill -f parameter_bridge || true
pkill -f bridge || true
pkill -f arena_spawn_panel.py || true
pkill -f arena_spawn_panel || true
pkill -f arena_obstacle_manager || true
pkill -f arena_roamer || true
pkill -f mecanum_kinematics || true
pkill -f odom_tf_broadcaster || true
pkill -f world_to_odom_publisher || true
pkill -f ros2 || true
sleep 3

echo "Display diagnostics:"
echo "DISPLAY=${DISPLAY:-<unset>}"
echo "XDG_SESSION_TYPE=${XDG_SESSION_TYPE:-<unset>}"

if [ "${XDG_SESSION_TYPE:-}" = "wayland" ]; then
    echo "Wayland session detected; forcing X11-compatible Qt/OpenGL settings."
    export QT_QPA_PLATFORM=xcb
    export MESA_GL_VERSION_OVERRIDE=3.3
fi

if command -v glxinfo >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    renderer="$(glxinfo 2>/dev/null | grep "OpenGL renderer" || true)"
    echo "${renderer:-OpenGL renderer: unavailable}"

    if echo "$renderer" | grep -Eiq 'llvmpipe|softpipe'; then
        echo "Software OpenGL renderer detected; forcing software rendering for stability."
        export LIBGL_ALWAYS_SOFTWARE=1
    fi
else
    echo "glxinfo unavailable or DISPLAY unset; defaulting to software rendering for headless launch."
    export LIBGL_ALWAYS_SOFTWARE=1
fi

# Provide safe defaults for environment variables that install/setup.bash expects
export COLCON_TRACE=${COLCON_TRACE:-0}
export AMENT_TRACE=${AMENT_TRACE:-}
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-python3}
export COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE:-/usr/bin/python3}
export COLCON_PREFIX_PATH=${COLCON_PREFIX_PATH:-}
export _CATKIN_SETUP_DIR=${_CATKIN_SETUP_DIR:-}
source install/setup.bash

# Additional OpenGL fixes for software rendering
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb}
export MESA_GL_VERSION_OVERRIDE=${MESA_GL_VERSION_OVERRIDE:-3.3}

# Configure for headless operation (no GUI components)
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
export GZ_HEADLESS=1

echo "Environment Configuration:"
echo "  QT_QPA_PLATFORM=$QT_QPA_PLATFORM"
echo "  MESA_GL_VERSION_OVERRIDE=$MESA_GL_VERSION_OVERRIDE"
echo "  LIBGL_ALWAYS_SOFTWARE=$LIBGL_ALWAYS_SOFTWARE"
echo "  GZ_HEADLESS=$GZ_HEADLESS"
echo ""

echo "Starting headless simulation..."
# Ensure Gazebo can resolve local model:// URIs
export GZ_SIM_RESOURCE_PATH="${BASE_DIR}/src/audix_pkg/models:${BASE_DIR}/src/audix_pkg:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="${BASE_DIR}/src/audix_pkg/models:${BASE_DIR}/src/audix_pkg:${IGN_GAZEBO_RESOURCE_PATH:-}"

# Launch with all GUI components disabled
ros2 launch src/audix_pkg/launch/full_mission.launch.py \
    use_rviz:=false \
    use_gazebo_gui:=false \
    use_spawn_panel:=false \
    auto_start:=true \
    world_name:=warehouse

echo "Headless simulation completed."
