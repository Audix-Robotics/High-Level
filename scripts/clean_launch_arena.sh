#!/usr/bin/env bash
set -euo pipefail

# Wrapper to ensure a single clean Gazebo+ROS2+RViz session.
# Usage: ./scripts/clean_launch_arena.sh

BASE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$BASE_DIR"

# ============================================================================
# STEP 1: Diagnose and configure display environment
# ============================================================================
echo "Display Environment Diagnostics:"
echo "  DISPLAY: ${DISPLAY:-<unset>}"
echo "  XDG_SESSION_TYPE: ${XDG_SESSION_TYPE:-<unset>}"

# Force X11-compatible Qt/OpenGL settings (essential for remote displays and Wayland)
export QT_QPA_PLATFORM=xcb
export MESA_GL_VERSION_OVERRIDE=3.3

# Check for software rendering and configure accordingly
if command -v glxinfo >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    renderer="$(glxinfo 2>/dev/null | grep "OpenGL renderer" || echo "unknown")"
    echo "  GPU Renderer: $renderer"
    
    if echo "$renderer" | grep -Eiq 'llvmpipe|softpipe'; then
        echo "  ⚠ Software OpenGL detected (softpipe/llvmpipe)"
        export LIBGL_ALWAYS_SOFTWARE=1
    fi
else
    echo "  Using failsafe: software rendering"
    export LIBGL_ALWAYS_SOFTWARE=1
fi
echo ""

# Provide safe defaults for environment variables that install/setup.bash expects
# to avoid "unbound variable" errors when running with `set -u`.
export COLCON_TRACE=${COLCON_TRACE:-0}
export AMENT_TRACE=${AMENT_TRACE:-}
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-python3}
export COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE:-/usr/bin/python3}
export COLCON_PREFIX_PATH=${COLCON_PREFIX_PATH:-}
export _CATKIN_SETUP_DIR=${_CATKIN_SETUP_DIR:-}
source install/setup.bash

# ============================================================================
# STEP 2: Clean up all lingering processes
# ============================================================================
echo "Killing lingering simulator/ROS processes..."
pkill -f gz || true
pkill -f gz_sim || true
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f rviz2 || true
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

sleep 1

# ============================================================================
# STEP 3: Set up display (create virtual framebuffer if needed)
# ============================================================================
if [ -z "${DISPLAY:-}" ]; then
    echo "No DISPLAY set, starting virtual framebuffer..."
    export DISPLAY=:99
    Xvfb :99 -screen 0 1024x768x24 > /dev/null 2>&1 &
    XVFB_PID=$!
    sleep 2
fi

echo "Environment Configuration for GUI:"
echo "  DISPLAY=$DISPLAY"
echo "  QT_QPA_PLATFORM=$QT_QPA_PLATFORM"
echo "  MESA_GL_VERSION_OVERRIDE=$MESA_GL_VERSION_OVERRIDE"
echo "  LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}"
echo ""

# Export Gazebo resource paths
export GZ_SIM_RESOURCE_PATH="${BASE_DIR}/src/audix_pkg/models:${BASE_DIR}/src/audix_pkg:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="${BASE_DIR}/src/audix_pkg/models:${BASE_DIR}/src/audix_pkg:${IGN_GAZEBO_RESOURCE_PATH:-}"

# ============================================================================
# STEP 4: Launch ROS2 simulation
# ============================================================================
echo "Starting ROS2 Gazebo simulation..."
ros2 launch src/audix_pkg/launch/full_mission.launch.py &
LAUNCH_PID=$!

# Ensure we clean up child processes on exit or interrupt
cleanup() {
  echo "Cleaning up launcher and rviz..."
  if [ -n "${RVIZ_PID:-}" ]; then
    kill "${RVIZ_PID}" 2>/dev/null || true
  fi
  if [ -n "${GUI_PID:-}" ]; then
    kill "${GUI_PID}" 2>/dev/null || true
  fi
  if [ -n "${LAUNCH_PID:-}" ]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
  fi
  if [ -n "${XVFB_PID:-}" ]; then
    kill "${XVFB_PID}" 2>/dev/null || true
  fi
  # Give processes a moment to exit, then force-kill lingering simulator processes
  sleep 1
  pkill -f gz || true
  pkill -f gz_sim || true
  pkill -f gzserver || true
  pkill -f gzclient || true
  pkill -f rviz2 || true
}
trap cleanup EXIT INT TERM

# Give simulator and bridges time to initialize before RViz
sleep 8

# Start the Start/Stop GUI so the robot remains in STOP until user presses START
echo "Starting start_stop_gui..."
python3 src/audix_pkg/scripts/start_stop_gui.py &
GUI_PID=$!

# Start a single RViz instance using the project's config
RVIZ_CONFIG=src/audix_pkg/rviz/full_mission.rviz
if [ ! -f "$RVIZ_CONFIG" ]; then
  echo "RViz config not found: $RVIZ_CONFIG"
else
  echo "Starting rviz2 with $RVIZ_CONFIG"
  rviz2 -d "$RVIZ_CONFIG" &
  RVIZ_PID=$!
fi

echo "Started: launch_pid=${LAUNCH_PID:-none} rviz_pid=${RVIZ_PID:-none} xvfb_pid=${XVFB_PID:-none}"

echo "Wrapper is running. To stop, Ctrl-C this script or kill the PIDs above."
wait
