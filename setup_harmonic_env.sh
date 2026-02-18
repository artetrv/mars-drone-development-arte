#!/usr/bin/env bash
# === ROS 2 Jazzy + Gazebo Harmonic sim env ===

# Base ROS
source /opt/ros/jazzy/setup.bash

# This workspace
source $HOME/harmonic_ws/install/setup.bash

# Clear any old Gazebo/Ignition vars from other projects
unset GZ_SIM_RESOURCE_PATH
unset GZ_SIM_SYSTEM_PLUGIN_PATH
unset GAZEBO_RESOURCE_PATH
unset GAZEBO_PLUGIN_PATH
unset IGN_GAZEBO_RESOURCE_PATH
unset IGN_GAZEBO_SYSTEM_PLUGIN_PATH

# Add only Harmonic resources we care about
#export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/harmonic_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
#export GZ_SIM_RESOURCE_PATH=$HOME/harmonic_ws/src/ardupilot_gazebo/models:$HOME/harmonic_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Models + worlds (order matters: custom → upstream → cache → system)
export GZ_SIM_RESOURCE_PATH="$HOME/harmonic_ws/src/tag_hover_two_tags/models:$HOME/harmonic_ws/src/tag_hover_sim/models:$HOME/harmonic_ws/src/tag_hover_sim/worlds:$HOME/harmonic_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/models:$HOME/harmonic_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/worlds:$HOME/.gz/models:/opt/ros/jazzy/share"

# ArduPilot Gazebo system plugins
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/harmonic_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo"


echo "[harmonic_ws] Env set. GZ_SIM_RESOURCE_PATH:"
echo "  $GZ_SIM_RESOURCE_PATH"
