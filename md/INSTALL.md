# Installation Guide

This guide walks through setting up the full simulation stack (ROS 2 Jazzy + Gazebo Harmonic + ArduPilot SITL) from a clean Ubuntu 24.04 machine, and optionally deploying to a Raspberry Pi companion computer.

**Target OS:** Ubuntu 24.04 LTS (Noble)  
**Architecture:** x86_64 (simulation), ARM64 (Pi 5 companion)

> **Current hardware (2026):** the companion computer is a **Raspberry Pi 5** running this full workspace directly with a **Luxonis OAK-D-LITE** camera — see `src/tag_hover_two_tags/launch/hardware_vision_stack_oak.launch.py`. Section 10 below describes the earlier Pi 4 + RealSense D455 deployment and is kept for reference.

---

## Table of Contents

1. [System prerequisites](#1-system-prerequisites)
2. [Install ROS 2 Jazzy](#2-install-ros-2-jazzy)
3. [Install Gazebo Harmonic](#3-install-gazebo-harmonic)
4. [Install ROS 2 + Gazebo bridge packages](#4-install-ros-2--gazebo-bridge-packages)
5. [Clone the repository](#5-clone-the-repository)
6. [Set up Python environment (ArduPilot SITL)](#6-set-up-python-environment-ardupilot-sitl)
7. [Build ArduPilot SITL](#7-build-ardupilot-sitl)
8. [Build the ROS 2 workspace](#8-build-the-ros-2-workspace)
9. [Verify the installation](#9-verify-the-installation)
10. [Pi companion setup (legacy: Pi 4 + D455)](#10-pi-companion-setup-legacy-pi-4--d455)

---

## 1. System prerequisites

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
  git curl wget build-essential cmake \
  python3-pip python3-venv python3-dev \
  libssl-dev libffi-dev
```

---

## 2. Install ROS 2 Jazzy

Follow the official guide: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Quick path:

```bash
# Add ROS 2 apt repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

Test:
```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

---

## 3. Install Gazebo Harmonic

Follow the official guide: https://gazebosim.org/docs/harmonic/install_ubuntu/

Quick path:

```bash
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

sudo apt update
sudo apt install -y gz-harmonic
```

Test:
```bash
gz sim --version   # should report Gazebo Harmonic
```

---

## 4. Install ROS 2 + Gazebo bridge packages

```bash
sudo apt install -y \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-mavros \
  ros-jazzy-mavros-extras \
  ros-jazzy-apriltag-ros \
  ros-jazzy-message-filters \
  ros-jazzy-tf2-ros \
  ros-jazzy-cv-bridge \
  python3-opencv

# Install GeographicLib datasets (required by MAVROS)
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

---

## 5. Clone the repository

```bash
git clone --recurse-submodules <repo-url> ~/your_ws
cd ~/your_ws
```

> Replace `~/your_ws` with whatever you want to name your workspace (e.g., `~/Mars-drone-development`). All scripts auto-detect the workspace root — the name does not matter.

If you already cloned without `--recurse-submodules`:
```bash
cd ~/your_ws
git submodule update --init --recursive
```

The three submodules are:
- `src/ardupilot` — ArduPilot firmware
- `src/ardupilot_gazebo` — Gazebo Harmonic plugin
- `src/gazebo_apriltag` — AprilTag Gazebo plugin

---

## 6. Set up Python environment (ArduPilot SITL)

ArduPilot SITL (`sim_vehicle.py`) needs a dedicated Python venv to avoid dependency conflicts with ROS 2.

```bash
cd ~/your_ws
python3 -m venv drone-venv
source drone-venv/bin/activate
pip install --upgrade pip
pip install mavproxy pymavlink
deactivate
```

> The venv must be at `drone-venv/` inside your workspace root (the QUICK_REFERENCE bringup commands activate it from that relative path).

---

## 7. Build ArduPilot SITL

```bash
cd ~/your_ws/src/ardupilot
source ../../drone-venv/bin/activate

# Install ArduPilot build dependencies
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile   # reload PATH

# Configure for Gazebo SITL
./waf configure --board sitl
./waf copter
```

This takes several minutes. The output binary goes to `src/ardupilot/build/sitl/bin/arducopter`.

---

## 8. Build the ROS 2 workspace

```bash
cd ~/your_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

> `--symlink-install` means edits to Python source files take effect immediately without rebuilding.

After building, source the workspace:
```bash
source install/setup.bash
```

Or use the convenience script (handles ROS + Gazebo env vars in one shot):
```bash
source setup_harmonic_env.sh
```

---

## 9. Verify the installation

Run these checks before attempting a full simulation launch.

```bash
cd ~/your_ws
source setup_harmonic_env.sh

# ROS 2 packages discoverable?
ros2 pkg list | grep tag_hover

# Gazebo resources reachable?
echo $GZ_SIM_RESOURCE_PATH

# ArduPilot SITL binary present?
ls src/ardupilot/build/sitl/bin/arducopter

# MAVROS node starts?
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14555@127.0.0.1:14550 &
sleep 3 && ros2 topic echo /mavros/state --once
```

### Full sim smoke test (two terminals)

**Terminal 1 — two-tag measurement (no SITL required):**
```bash
cd ~/your_ws && source setup_harmonic_env.sh
ros2 launch tag_hover_two_tags sim_vision_stack.launch.py
```

**Terminal 2:**
```bash
cd ~/your_ws && source setup_harmonic_env.sh
ros2 launch tag_hover_two_tags sim_lockon_backbone.launch.py
```

You should see Gazebo open with two tags, and `/relative_vibration_pose` publishing.

For the full 7-terminal SITL bringup (ArduPilot + MAVROS + hover controller), see:
```
src/tag_hover_sim/QUICK_REFERENCE.md
```

---

## 10. Pi companion setup (legacy: Pi 4 + D455)

> **Legacy section.** The current companion is a Pi 5 running the full workspace with the OAK-D-LITE camera (see note at the top of this guide) — the steps in sections 1–9 cover it. The instructions below apply to the earlier Pi 4 + RealSense D455 setup.

The Pi 4 runs a separate lightweight workspace (`~/drone-pi/`) with a single package: `tag_hover_controller`.

### Prerequisites (on Pi)

```bash
# Ubuntu 24.04 Server (ARM64) with ROS 2 Jazzy installed
sudo apt install -y \
  ros-jazzy-mavros ros-jazzy-mavros-extras \
  ros-jazzy-apriltag-ros ros-jazzy-tf2-ros \
  ros-jazzy-cv-bridge python3-opencv

sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# RealSense SDK + ROS wrapper
sudo apt install -y ros-jazzy-realsense2-camera
```

### Clone and build the Pi workspace

```bash
# On the Pi (SSH: ssh mars@<pi-ip>)
git clone <repo-url> ~/drone-pi
cd ~/drone-pi
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tag_hover_controller --symlink-install
source install/setup.bash
```

### Camera (RealSense D455)

Launch at 640×480×15 fps with IMU/tf/sync disabled (confirmed safe at Pi 4 CPU load):
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=false \
  enable_infra1:=false enable_infra2:=false \
  enable_accel:=false enable_gyro:=false \
  publish_tf:=false enable_sync:=false \
  rgb_camera.color_profile:=640x480x15
```

> **Important:** Use `rs_launch.py`, not `ros2 run realsense2_camera_node` — the node launcher does not reliably apply camera profile parameters.

> Valid fps values at 640×480: **6, 15, 30 only.** Values 10 and 12 silently fall back to 1280×720×30.

### Run the controller

```bash
source /opt/ros/jazzy/setup.bash && source ~/drone-pi/install/setup.bash
ros2 run tag_hover_controller hover_guided_hold --ros-args \
  -p camera_frame:=camera_color_optical_frame \
  -p target_distance:=1.5 \
  -p rate_hz:=10.0 \
  -p use_sim_time:=false \
  -p k_vert:=0.1 \
  -p vert_tol:=0.2
```

FCU must be in **GUIDED** mode (optical flow active). Serial: `/dev/ttyS0` at 57600 baud.

Full Pi 4 runbook: `docs/PI4_RUNBOOK.md` (gitignored — kept on dev machine only).
