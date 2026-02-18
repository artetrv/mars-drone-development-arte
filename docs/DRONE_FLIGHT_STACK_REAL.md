# Drone Flight Stack (Real Hardware) — Student Deployment Guide

**Purpose:** Deploy the AprilTag hover/search control stack (tested in simulation) to real Raspberry Pi 5 + Pixhawk hardware.

**Last Updated:** January 22, 2026 (Based on verified simulation implementation)

---

## 1. System Overview

**Hardware:**
- Raspberry Pi 5 (companion computer)
- Pixhawk flight controller (ArduPilot firmware)
- USB camera (V4L2 compatible)
- AprilTag target (36h11 family, physical size must be known)

**Control Pipeline:**
```
USB Camera → ROS2 camera driver → /image_raw + /camera_info
                ↓
         apriltag_ros detector → /detections
                ↓
    apriltag_pnp_broadcaster → TF (camera → tag36h11:X)
                ↓
       hover_yaw_search controller → velocity commands
                ↓
            MAVROS → Pixhawk (serial)
```

**Controller Behavior:**
- **SEARCH mode:** Constant yaw rotation until AprilTag detected, then auto-lock
- **LOCK mode:** 4-DOF visual servoing (yaw alignment, distance regulation, lateral centering, altitude hold disabled in Phase 1)
- Controller automatically switches from SEARCH→LOCK when tag appears in camera

---

## 2. Key Improvements from Simulation (What's Changed)

The simulation version of `hover_yaw_search.py` has been significantly enhanced with:

### Phase 1 Controller Architecture (Current - STABLE)
✅ **Camera-frame control:** All errors computed in camera optical frame before body transformation
✅ **Proper distance control:** Uses camera Z-axis (not body X) for range regulation  
✅ **Correct yaw alignment:** Uses `atan2(x_cam, z_cam)` for optical axis alignment
✅ **Auto-lock behavior:** Automatically locks when tag detected (works in both SEARCH and LOCK modes)
✅ **Alignment gating:** Only translates when yaw error < threshold to avoid fighting maneuvers
✅ **Independent P controllers:** Separate gains for yaw, distance, lateral, vertical (vertical disabled in Phase 1)

### Files to Copy from Simulation to Hardware

**CRITICAL:** The simulation versions are NEWER and BETTER than any old hardware versions. You MUST copy these files:

1. **`~/harmonic_ws/src/tag_hover_sim/tag_hover_sim/hover_yaw_search.py`**  
   - Phase 1 camera-frame controller (verified stable)
   - Auto-lock SEARCH→LOCK behavior
   - Proper distance + yaw control
   
2. **`~/harmonic_ws/src/tag_hover_sim/tag_hover_sim/apriltag_pnp_broadcaster.py`**  
   - Refined solvePnP with IPPE_SQUARE + ITERATIVE fallback
   - Proper quaternion math
   - More stable than detector's built-in TF

3. **`~/harmonic_ws/src/tag_hover_sim/config/apriltag_params.yaml`**  
   - Detector configuration (tag family, size, etc.)
   - **IMPORTANT:** Update `tag_size` parameter to match YOUR physical tag size

---

## 3. Pre-Flight Setup on Raspberry Pi

### 3.1 Install ROS2 Jazzy (if not already installed)
```bash
# Follow official ROS2 Jazzy installation for Ubuntu
# Install ros-jazzy-desktop or ros-jazzy-ros-base
```

### 3.2 Install Required Packages
```bash
sudo apt install -y \
  ros-jazzy-mavros \
  ros-jazzy-mavros-extras \
  ros-jazzy-v4l2-camera \
  ros-jazzy-apriltag-ros \
  ros-jazzy-tf2-ros \
  python3-pip \
  python3-opencv

# Install MAVROS GeographicLib datasets (REQUIRED for GPS)
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

### 3.3 Set Up Workspace on Raspberry Pi
```bash
cd ~
mkdir -p drone_ws/src/tag_hover_sim
cd drone_ws/src/tag_hover_sim

# Create package structure
mkdir -p tag_hover_sim config
touch tag_hover_sim/__init__.py
```

### 3.4 Copy Files from Simulation (YOUR ACTION REQUIRED)

**From your simulation workspace** (`~/harmonic_ws/`), copy these EXACT files:

```bash
# On simulation computer, create transfer package:
cd ~/harmonic_ws/src/tag_hover_sim
tar -czf ~/tag_hover_hardware.tar.gz \
  tag_hover_sim/hover_yaw_search.py \
  tag_hover_sim/apriltag_pnp_broadcaster.py \
  config/apriltag_params.yaml \
  package.xml \
  setup.py \
  setup.cfg \
  resource/tag_hover_sim

# Transfer to Raspberry Pi (use scp, USB drive, etc.)
# Then on Raspberry Pi:
cd ~/drone_ws/src/tag_hover_sim
tar -xzf ~/tag_hover_hardware.tar.gz
```

### 3.5 CRITICAL: Update AprilTag Size
```bash
# Edit the config file on Raspberry Pi
nano ~/drone_ws/src/tag_hover_sim/config/apriltag_params.yaml
```

**Find this line:**
```yaml
tag_size: 0.0376  # <-- CHANGE THIS to your physical tag size in meters
```

**Measure your physical AprilTag** (black border to black border) and update. Examples:
- Small tag: `0.0376` m (3.76 cm)
- Medium tag: `0.162` m (16.2 cm)  
- Large tag: `0.25` m (25 cm)

### 3.6 Build the Package
```bash
cd ~/drone_ws
colcon build --packages-select tag_hover_sim
source install/setup.bash

# Add to .bashrc for persistence
echo "source ~/drone_ws/install/setup.bash" >> ~/.bashrc
```

### 3.7 Configure Serial Port (Pixhawk Connection)
```bash
# Find Pixhawk serial port
ls /dev/ttyACM* /dev/ttyAMA*

# Common ports:
# - /dev/ttyAMA0 (GPIO UART on Pi 5)
# - /dev/ttyACM0 (USB connection)

# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# REBOOT required for group change
sudo reboot
```

---

## 4. Flight Operations (Step-by-Step)

### Terminal 1: Start MAVProxy (Serial Owner)
```bash
# MAVProxy owns the serial port and bridges to UDP for MAVROS
mavproxy.py --master=/dev/ttyAMA0 --baudrate=57600 --out=udp:127.0.0.1:14555

# This allows MAVROS to connect via UDP (see Terminal 1b below)
# You can also use MAVProxy for arming/mode changes/takeoff
```

**Verify:** You should see heartbeat messages and `online system 1`.

### Terminal 1b: Start MAVROS (UDP Client)
```bash
source ~/drone_ws/install/setup.bash

# MAVROS connects to MAVProxy's UDP output
ros2 launch mavros apm.launch fcu_url:=udp://:14555@127.0.0.1:14550
```

**Verify:** You should see `CON: Got HEARTBEAT` messages.

### Terminal 2: Start Camera Driver
```bash
source ~/drone_ws/install/setup.bash

# V4L2 camera (adjust video_device to your camera)
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video4 \
  -p "image_size:='[1280,720]" \
  -p time_per_frame.num:=1 \
  -p time_per_frame.den:=30 \
  -p pixel_format:=YUYV \
  -p output_encoding:=rgb8 \
  -p frame_id:=camera \
  -p camera_info_url:=file:///home/mars/.ros/camera_info/camera_1280x720.yaml

# Verify topics exist:
# ros2 topic hz /image_raw        # Should show ~30 Hz
# ros2 topic hz /camera_info      # Should show ~30 Hz
```

### Terminal 3: Start AprilTag Detector
```bash
source ~/drone_ws/install/setup.bash

ros2 run apriltag_ros apriltag_node --ros-args \
  --params-file ~/drone_ws/install/tag_hover_sim/share/tag_hover_sim/config/apriltag_params.yaml \
  -r image_rect:=/image_raw \
  -r camera_info:=/camera_info

# Verify detections (point camera at AprilTag):
# ros2 topic echo /detections --no-arr
```

### Terminal 4: Start PnP TF Broadcaster
```bash
source ~/drone_ws/install/setup.bash

ros2 run tag_hover_sim apriltag_pnp_broadcaster --ros-args \
  -p camera_frame:=camera \
  -p tag_prefix:=tag36h11 \
  -p tag_size_m:=0.162 \
  -p camera_info_topic:=/camera_info \
  -p detections_topic:=/detections

# IMPORTANT: tag_size_m MUST match your physical tag size!
# Verify TF (point camera at tag):
# ros2 run tf2_ros tf2_echo camera tag36h11:0
```

### Terminal 5: Start Hover Controller
```bash
source ~/drone_ws/install/setup.bash

# Start in SEARCH mode (will auto-lock when tag detected)
ros2 run tag_hover_controller hover_yaw_search --ros-args \
  -p mode:=SEARCH \
  -p rate_hz:=20.0 \
  -p camera_frame:=camera \
  -p body_frame:=base_link \
  -p tag_frame:=tag36h11:0 \
  -p search_yaw:=0.25 \
  -p lock_k_yaw:=0.1 \
  -p lock_k_distance:=0.2 \
  -p lock_k_lateral:=0.1 \
  -p target_distance:=2.0 \
  -p yaw_align_threshold:=0.1 \
  -p mavros_prefix:=/mavros

# Controller will log:
# - "Waiting for MAVROS..." (until connected)
# - "MAVROS connected and ready"
# - "SEARCH->TAG FOUND" when tag detected (auto-lock engages)
# - "LOCK [Phase1-CamFrame]: yaw_err=... yaw_cmd=..." when locked
```

### Manual Flight Commands (Terminal 6)
```bash
# MAVProxy is already running in Terminal 1
# Use that terminal's console for commands:

# For optical flow (no GPS) drone, use GUIDED_NOGPS:
mode GUIDED_NOGPS
arm throttle
takeoff 1

# Controller will now be active!
# Watch Terminal 5 for controller status

# To land:
mode LAND

# Emergency:
mode STABILIZE  # Returns manual control to RC

# Alternative: Use Mission Planner
# Connect via UDP to Pi_IP:14550 (MAVProxy bridges to it)
```

---

## 5. Parameter Tuning Guide

If the drone's behavior is unstable, adjust these parameters:

### 5.1 Yaw Control (spinning too fast/slow)
```bash
# Reduce if drone spins too aggressively:
-p lock_k_yaw:=0.05  # (default: 0.1)

# Increase if drone doesn't align fast enough:
-p lock_k_yaw:=0.15

# Clamp maximum yaw rate:
-p max_yaw_rate:=0.3  # (default: 0.6 rad/s)
```

### 5.2 Distance Control (approaching too fast/slow)
```bash
# Reduce if drone rushes toward tag:
-p lock_k_distance:=0.1  # (default: 0.2)

# Increase if drone doesn't approach:
-p lock_k_distance:=0.3

# Clamp maximum forward velocity:
-p max_forward_vel:=0.3  # (default: 0.5 m/s)
```

### 5.3 Lateral Control (side-to-side oscillation)
```bash
# Reduce if drone oscillates left/right:
-p lock_k_lateral:=0.05  # (default: 0.1)

# Clamp maximum lateral velocity:
-p max_lateral_vel:=0.3  # (default: 0.5 m/s)
```

### 5.4 Target Distance (standoff range)
```bash
# Adjust how far from tag drone should stabilize:
-p target_distance:=3.0  # (default: 2.0 meters)
```

### 5.5 Yaw Alignment Threshold
```bash
# Only move forward/lateral when yaw error is small:
-p yaw_align_threshold:=0.15  # (default: 0.1 radians ≈ 5.7 degrees)
```

---

## 6. Troubleshooting

### Camera not publishing
```bash
# List available cameras:
v4l2-ctl --list-devices

# Test camera:
ros2 run v4l2_camera v4l2_camera_node

# Check topics:
ros2 topic list | grep -E "image|camera_info"
```

### MAVROS not connecting
```bash
# Check serial permissions:
ls -l /dev/ttyAMA0  # Should show dialout group
groups              # Should include dialout

# Check baud rate matches Pixhawk SERIAL1_BAUD parameter

# Test serial connection:
sudo screen /dev/ttyAMA0 921600  # Should see MAVLink traffic
```

### AprilTag not detected
```bash
# Check tag size in apriltag_params.yaml matches physical tag
# Ensure good lighting (no glare, shadows)
# Verify tag family (36h11 is most common)
# Check detector is running:
ros2 topic echo /detections --no-arr
```

### Controller not locking
```bash
# Verify TF is being published:
ros2 run tf2_ros tf2_echo camera tag36h11:0

# Check controller is subscribed:
ros2 node info /hover_yaw_search

# Ensure camera_frame parameter matches TF tree
```

### Drone drifts away
```bash
# Phase 1 controller does NOT control position (only yaw, distance, lateral)
# FCU must be in GUIDED or LOITER mode for position hold
# Ensure GPS lock is good (if using GPS-based modes)
```

---

## 7. Safety Checklist

**BEFORE EVERY FLIGHT:**
- [ ] All 5 terminals running (MAVROS, camera, detector, broadcaster, controller)
- [ ] `ros2 topic echo /detections` shows tag when pointed at it
- [ ] `ros2 run tf2_ros tf2_echo camera tag36h11:0` shows stable TF
- [ ] Controller logs "MAVROS connected and ready"
- [ ] RC transmitter in hand with mode switch ready
- [ ] Clear flight area (no obstacles in 10m radius)
- [ ] AprilTag mounted stable and vertical
- [ ] Battery fully charged
- [ ] Propellers in good condition

**DURING FLIGHT:**
- [ ] Monitor controller terminal for errors
- [ ] Keep RC mode switch ready to switch to STABILIZE (manual override)
- [ ] Watch for oscillations (reduce gains if observed)
- [ ] Never fly beyond visual line of sight

**EMERGENCY PROCEDURES:**
- **Unstable behavior:** Immediately switch to STABILIZE mode on RC
- **Lost connection:** RTL (Return to Launch) mode will activate
- **Controller crash:** Drone will hold last command for ~2 seconds, then timeout

---

## 8. File Reference

### Files to Copy FROM Simulation
| File | Source (simulation) | Destination (hardware) |
|------|-------------------|----------------------|
| `hover_yaw_search.py` | `~/harmonic_ws/src/tag_hover_sim/tag_hover_sim/` | `~/drone_ws/src/tag_hover_sim/tag_hover_sim/` |
| `apriltag_pnp_broadcaster.py` | `~/harmonic_ws/src/tag_hover_sim/tag_hover_sim/` | `~/drone_ws/src/tag_hover_sim/tag_hover_sim/` |
| `apriltag_params.yaml` | `~/harmonic_ws/src/tag_hover_sim/config/` | `~/drone_ws/src/tag_hover_sim/config/` |
| `package.xml` | `~/harmonic_ws/src/tag_hover_sim/` | `~/drone_ws/src/tag_hover_sim/` |
| `setup.py` | `~/harmonic_ws/src/tag_hover_sim/` | `~/drone_ws/src/tag_hover_sim/` |
| `setup.cfg` | `~/harmonic_ws/src/tag_hover_sim/` | `~/drone_ws/src/tag_hover_sim/` |

### Files to Edit on Hardware
1. **`config/apriltag_params.yaml`** - Update `tag_size` to match physical tag
2. **Terminal commands** - Update `/dev/ttyAMA0` and baud rate to match your setup

---

## 9. Quick Reference Commands

**Start all nodes (6 terminals):**
```bash
# Terminal 1: MAVProxy (Serial Owner)
mavproxy.py --master=/dev/ttyAMA0 --baudrate=57600 --out=udp:127.0.0.1:14555

# Terminal 1b: MAVROS (UDP Client)
ros2 launch mavros apm.launch fcu_url:=udp://:14555@127.0.0.1:14550

# Terminal 2: Camera
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video4 -p "image_size:='[1280,720]" -p time_per_frame.num:=1 -p time_per_frame.den:=30 -p pixel_format:=YUYV -p output_encoding:=rgb8 -p frame_id:=camera -p camera_info_url:=file:///home/mars/.ros/camera_info/camera_1280x720.yaml

# Terminal 3: Detector
ros2 run apriltag_ros apriltag_node --ros-args --params-file ~/drone_ws/install/tag_hover_sim/share/tag_hover_sim/config/apriltag_params.yaml -r image_rect:=/image_raw -r camera_info:=/camera_info

# Terminal 4: TF Broadcaster
ros2 run tag_hover_sim apriltag_pnp_broadcaster --ros-args -p camera_frame:=camera -p tag_size_m:=0.162

# Terminal 5: Controller
ros2 run tag_hover_controller hover_yaw_search --ros-args -p mode:=SEARCH -p rate_hz:=20.0
```

**Verification commands:**
```bash
ros2 topic hz /image_raw                    # Camera publishing
ros2 topic echo /detections --no-arr        # Tag detected
ros2 run tf2_ros tf2_echo camera tag36h11:0 # TF stable
ros2 topic echo /mavros/state               # MAVROS connected
```

---

## 10. Known Limitations (Phase 1)

1. **No altitude control:** Vertical DOF disabled; FCU maintains altitude
2. **No position hold:** Only yaw/distance/lateral controlled; FCU must provide position stability
3. **Single tag:** Only tracks `tag36h11:0`; multi-tag support not implemented
4. **No obstacle avoidance:** Pilot must ensure clear flight path
5. **Fixed gains:** No adaptive control; requires manual tuning per drone

---

**Questions? Issues? Check simulation logs in `~/harmonic_ws/docs/PROGRESS_LOG.md` for debugging hints.**
