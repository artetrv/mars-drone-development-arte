# ArduPilot SITL Setup (harmonic_ws)

Stack: ROS 2 Jazzy + Gazebo Harmonic + ArduPilot SITL + MAVROS  
Paths below use `~/harmonic_ws` (this repo).

## 1) Prepare Python environment

The workspace uses `drone-venv/` at the repo root for ArduPilot SITL dependencies. Create it once:
```bash
cd ~/harmonic_ws
python3 -m venv drone-venv
touch drone-venv/COLCON_IGNORE   # prevents colcon from scanning the venv
source drone-venv/bin/activate
```

Notes:
- Do not use `--user` inside a venv.
- The `COLCON_IGNORE` file is required — without it, colcon errors when trying to scan the venv during builds.

## 2) Install prerequisites and MAVProxy
```bash
source ~/harmonic_ws/drone-venv/bin/activate
cd ~/harmonic_ws/src/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
pip install --upgrade pip
pip install MAVProxy
```

## 3) Install ArduPilot Gazebo plugin (if missing)
If Gazebo can’t load `libArduPilotPlugin.so`, build and install it:
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
sudo make install
```
Export plugin paths before launching Gazebo:
```bash
export GZ_PLUGIN_PATH=/usr/local/lib:$GZ_PLUGIN_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/local/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

## 4) Start ArduPilot SITL (Copter + Gazebo profile)
```bash
cd ~/harmonic_ws/src/ardupilot
source venv/bin/activate
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```
This enables the FDM on 127.0.0.1 using UDP ports 9002 (in) / 9003 (out), matching our model.

## 5) Launch Gazebo with our models/worlds
Run from this repo (after build, or directly from source):
```bash
cd ~/harmonic_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Ensure local models are visible (source + install)
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/harmonic_ws/src/ardupilot_gazebo/models:$HOME/harmonic_ws/src/tag_hover_sim/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/harmonic_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/models:$HOME/harmonic_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/worlds

# Example: run AprilTag world directly
gz sim -r src/tag_hover_sim/worlds/apriltag_test.sdf
```

## 6) MAVROS (single UAV)
Typical ArduPilot SITL defaults: send 14550, listen 14555. Adjust if you set different `--out` ports.
```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14555@127.0.0.1:14550
ros2 topic echo /mavros/state --once
```
If you run MAVROS from `sim_lockon_backbone.launch.py`, do not start a second MAVROS in the same ROS domain; otherwise give it a unique `__node` and `namespace`.

## 7) Verification
- Gazebo log shows ArduPilot plugin loaded and connected
- `ss -u -nlp | grep -E '9002|9003|14550'` shows ports in use
- MAVROS `/mavros/state` reports `connected: true` when started

## 8) Troubleshooting
- Missing requirements.txt when pip installing: expected; use `install-prereqs-ubuntu.sh` and `pip install MAVProxy` in venv.
- `pip --user` inside venv: don’t use `--user`; install into venv.
- Plugin not found: build/install `ardupilot_gazebo` and export GZ plugin paths.
- Model not found: ensure `GZ_SIM_RESOURCE_PATH` includes this repo’s model paths (src + install).
- Two-vehicle setup: requires distinct FDM ports per model and two SITL instances; not yet scripted here.

## Status & next steps
- Single-UAV ArduPilot world and launch added; verified build/install steps documented.
- Next: add `mavros_ardupilot.launch.py` (namespaced), and a dual-UAV ArduPilot setup with distinct ports and two SITL processes.
