#!/usr/bin/env bash
# Run ArduPilot SITL for Gazebo Harmonic integration.
# Source this from your workspace root.

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$WS_ROOT/drone-venv/bin/activate"

cd "$WS_ROOT/src/ardupilot/Tools/autotest" || exit 1

./sim_vehicle.py \
  -v ArduCopter \
  -f gazebo-iris \
  --model JSON \
  --console \
  --map \
  --out=127.0.0.1:14550
