# ALL WORK SUMMARY — harmonic_ws

Last updated: 2026-02-16  
Stack: ROS 2 Jazzy + Gazebo Harmonic + ArduPilot SITL + MAVROS

## Purpose
This workspace focuses on simulating and exercising AprilTag-based hover/yaw-search behavior with ArduPilot SITL in Gazebo Harmonic, mirroring the real Pixhawk + companion workflow. It also contains ArduPilot Gazebo models and worlds used by the simulation.

## Key components
- `src/tag_hover_sim/` — AprilTag hover/yaw-search simulation package.
  - Launch: `launch/sim_lockon_backbone.launch.py` (MAVROS + controller).
  - Models: `iris_with_ardupilot`, `iris_with_rgb_camera`, `gimbal_small_3d_fixed`.
  - Worlds: `apriltag_test.sdf`, `testing_nodrone.sdf`.
  - Config: `config/apriltag_params.yaml`.
- `src/tag_hover_two_tags/` — Two-tag relative pose measurement stack.
  - Launch: `launch/sim_vision_stack.launch.py`, `launch/sim_lockon_backbone.launch.py`.
  - Worlds: `apriltag_two_tags.sdf`.
  - Config: `config/apriltag_params.yaml`.
  - Notes: 5-inch tags (0.127 m), overlay node available, oscillator launch added.
- `src/ardupilot_gazebo/` — ArduPilot models/worlds installed into `install/ardupilot_gazebo/share/...`.
- Root docs: `START_HERE.md`, `LOCKON_NOTES.md`, `ardupilot_setup.md`, `flight_guide.md`, `code_index.md`, `DRONE_FLIGHT_STACK_REAL.md`.

## How to run (single UAV, Tag Hover Sim)
1) Start ArduPilot SITL (example ports): `sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out=127.0.0.1:14550 --out=127.0.0.1:14555`

./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
2) Launch Gazebo world: `gz sim -r src/tag_hover_sim/worlds/apriltag_test.sdf` (with `GZ_SIM_RESOURCE_PATH` pointing to `src/.../models` and `install/.../models`).
3) Launch MAVROS + controller: `ros2 launch tag_hover_sim sim_lockon_backbone.launch.py fcu_url:=udp://:14555@127.0.0.1:14550`
4) Bridge camera → ROS, run `apriltag_ros` with `config/apriltag_params.yaml`, run PnP TF broadcaster.
5) Check `/mavros/state` (connected, GUIDED/LOITER), arm, take off; SEARCH rotates, LOCK centers on tag.

## Common pitfalls
- Two MAVROS instances in the same ROS domain cause crashes; run only one (or unique name/ns).
- Missing models: set `GZ_SIM_RESOURCE_PATH` to include `src/ardupilot_gazebo/models` and `src/tag_hover_sim/models` (and their install paths).
- ArduPilot plugin missing: build/install `ardupilot_gazebo` and export plugin paths.

## Where to read next
- `START_HERE.md` — entrypoint and links.
- `LOCKON_NOTES.md` — detailed lockon/AprilTag flow and resource paths.
- `TWO_TAG_NOTES.md` — two-tag measurement notes and launch flow.
- `ardupilot_setup.md` — SITL + plugin setup.
- `flight_guide.md` — flying steps and common warnings.
- `code_index.md` — map of packages/launches/models/topics.
- `DRONE_FLIGHT_STACK_REAL.md` — real hardware stack overview.
