#!/usr/bin/env bash
# === ROS 2 Jazzy + Gazebo Harmonic sim env ===
# Source this from your workspace root, or let it auto-detect.

# Resolve workspace root from this script's location (works regardless of clone name/path)
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Base ROS
source /opt/ros/jazzy/setup.bash

# This workspace
source "$WS_ROOT/install/setup.bash"

# Clear any old Gazebo/Ignition vars from other projects
unset GZ_SIM_RESOURCE_PATH
unset GZ_SIM_SYSTEM_PLUGIN_PATH
unset GAZEBO_RESOURCE_PATH
unset GAZEBO_PLUGIN_PATH
unset IGN_GAZEBO_RESOURCE_PATH
unset IGN_GAZEBO_SYSTEM_PLUGIN_PATH

# Add only Harmonic resources we care about
#export GZ_SIM_SYSTEM_PLUGIN_PATH=$WS_ROOT/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
#export GZ_SIM_RESOURCE_PATH=$WS_ROOT/src/ardupilot_gazebo/models:$WS_ROOT/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Models + worlds (order matters: custom → upstream → cache → system)
export GZ_SIM_RESOURCE_PATH="$WS_ROOT/src/tag_hover_two_tags/models:$WS_ROOT/src/tag_hover_sim/models:$WS_ROOT/src/tag_hover_sim/worlds:$WS_ROOT/install/ardupilot_gazebo/share/ardupilot_gazebo/models:$WS_ROOT/install/ardupilot_gazebo/share/ardupilot_gazebo/worlds:$HOME/.gz/models:/opt/ros/jazzy/share"

# ArduPilot Gazebo system plugins
export GZ_SIM_SYSTEM_PLUGIN_PATH="$WS_ROOT/install/ardupilot_gazebo/lib/ardupilot_gazebo"

# Keep ROS 2 DDS traffic on localhost only.
# Prevents VS Code from seeing/forwarding DDS ports across the WSL bridge,
# which causes severe Gazebo slowdowns (observed: RTF drops to 0.5%).
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1

echo "[ws] Env set. WS_ROOT=$WS_ROOT"
echo "  GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
