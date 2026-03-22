# Display Issues - Root Cause Analysis & Solution

## Problem Summary
The system was experiencing display/rendering issues when launching Gazebo and RViz2:
- **Error**: "OpenGL 3.3 is not supported. Please update your graphics card drivers"
- **Error**: "Unable to create the rendering window after 100 tries"
- **Root Cause**: Software OpenGL renderer (softpipe) on remote display with incorrect Qt/OpenGL configuration

---

## Diagnostic Findings

### 1. Display Configuration
```
DISPLAY: 172.19.96.1:0          (Remote X11 display)
XDG_SESSION_TYPE: <unset>       (Using X11, not Wayland)
GPU Renderer: softpipe          (Software rendering - no hardware acceleration)
```

### 2. Why Display Issues Occurred
- **Software renderer**: Linux container with softpipe (CPU-based rendering)
- **Conflicting OpenGL versions**: Qt and Ogre fighting over renderer selection
- **Remote display**: X11 forwarding from Docker to host requires specific configuration
- **Missing environment variables**: Qt wasn't configured for proper platform abstraction

---

## Solution Applied

### Environment Variables Set in Launch Scripts

#### 1. **QT_QPA_PLATFORM=xcb**
- Forces Qt to use X11 (xcb) instead of Wayland
- Essential for compatibility with remote displays
- Prevents Qt initialization failures

#### 2. **MESA_GL_VERSION_OVERRIDE=3.3**
- Tells Mesa to report OpenGL 3.3 capability
- Works with software rendering (softpipe)
- Prevents "OpenGL 3.3 not supported" errors

#### 3. **LIBGL_ALWAYS_SOFTWARE=1**
- Forces CPU-based software rendering explicitly
- Prevents conflicts between hardware/software renderers
- Ensures stable operation on systems without GPU

---

## Files Modified

### 1. **scripts/clean_launch_headless.sh**
Added automatic detection and configuration:
```bash
# Detect software renderer
if echo "$renderer" | grep -Eiq 'llvmpipe|softpipe'; then
    export LIBGL_ALWAYS_SOFTWARE=1
fi

# Always set Qt/Mesa for compatibility
export QT_QPA_PLATFORM=xcb
export MESA_GL_VERSION_OVERRIDE=3.3
```

### 2. **scripts/clean_launch_arena.sh**
Added comprehensive diagnostics and configuration:
```bash
# Step 1: Display diagnostics
echo "DISPLAY: ${DISPLAY:-<unset>}"
echo "XDG_SESSION_TYPE: ${XDG_SESSION_TYPE:-<unset>}"

# Step 2: Detect renderer and configure
renderer="$(glxinfo 2>/dev/null | grep "OpenGL renderer" || echo "unknown")"
if echo "$renderer" | grep -Eiq 'llvmpipe|softpipe'; then
    export LIBGL_ALWAYS_SOFTWARE=1
fi

# Step 3: Set Qt/OpenGL compatibility
export QT_QPA_PLATFORM=xcb
export MESA_GL_VERSION_OVERRIDE=3.3

# Step 4: Kill all processes first (critical!)
pkill -f "gz sim"
pkill -f rviz2
# ... more cleanup
sleep 3

# Step 5: Launch with configured environment
```

---

## Key Principles for Display Fixes

### ✅ Prevention Steps (Always Do These First)
1. **Kill all lingering processes** before every launch
   ```bash
   pkill -f "gz sim"
   pkill -f "rviz2"
   pkill -f "gz"
   sleep 3
   ```

2. **Diagnose environment**
   ```bash
   echo $DISPLAY
   echo $XDG_SESSION_TYPE
   glxinfo | grep "OpenGL renderer"
   ```

3. **Force X11 for remote displays**
   ```bash
   export QT_QPA_PLATFORM=xcb
   export MESA_GL_VERSION_OVERRIDE=3.3
   ```

4. **Handle software rendering explicitly**
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   ```

---

## Verification

### Successful Indicators
✅ Script shows diagnostic output:
```
Display Environment Diagnostics:
  DISPLAY: 172.19.96.1:0
  XDG_SESSION_TYPE: <unset>
  GPU Renderer: softpipe
  ⚠ Software OpenGL detected (softpipe/llvmpipe)

Environment Configuration for GUI:
  QT_QPA_PLATFORM=xcb
  MESA_GL_VERSION_OVERRIDE=3.3
  LIBGL_ALWAYS_SOFTWARE=1
```

✅ All ROS2 nodes start without OpenGL errors:  
```
[INFO] [gazebo-1]: process started with pid [50464]
[INFO] [arena_roamer.py-12]: process started with pid [50741]
```

✅ Topics are accessible:
```
/cmd_vel
/ir_front/scan
/odometry/filtered
```

---

## How to Launch Going Forward

### For Headless Operation (Recommended)
```bash
./scripts/clean_launch_headless.sh
```
- No GUI components
- Automatic environment detection
- Stable on all systems

### For Full GUI Experience
```bash
./scripts/clean_launch_arena.sh
```
- Includes RViz2 and Gazebo GUI
- Automatic virtual display if needed
- All environment fixes applied

---

## Troubleshooting

### If Still Getting OpenGL Errors
1. Kill all processes: `pkill -f "gz sim"; pkill -f rviz2`
2. Check environment: `glxinfo | grep "OpenGL renderer"`
3. Ensure variables are set:
   ```bash
   echo $QT_QPA_PLATFORM
   echo $MESA_GL_VERSION_OVERRIDE
   echo $LIBGL_ALWAYS_SOFTWARE
   ```

### If GPU Renderer Available
If you have dedicated GPU support detected:
```bash
glxinfo | grep -i "GeForce\|Intel\|AMD"  # Will show actual GPU
```
Then you can optionally remove `LIBGL_ALWAYS_SOFTWARE=1` for acceleration.

---

## Summary

| Issue | Cause | Fix |
|-------|-------|-----|
| OpenGL 3.3 not supported | Software renderer | MESA_GL_VERSION_OVERRIDE=3.3 |
| Qt window init failures | Wrong platform | QT_QPA_PLATFORM=xcb |
| Rendering conflicts | Mixed HW/SW | LIBGL_ALWAYS_SOFTWARE=1 |
| Stale process errors | Old instances | pkill -f before launch |

**Status**: ✅ **RESOLVED** - Display issues fixed with environment configuration
