#!/usr/bin/env bash
# Gazebo Harmonic rail: gz physics world + ArduPilot SITL (JSON backend) +
# Guardrail demo over pymavlink.
# Usage (WSL):  bash run_gazebo_demo.sh [on|off] [--dynamic]
set -e
SHIELD="${1:-on}"
DYN="${2:-}"
DIR="$(cd "$(dirname "$0")" && pwd)"

# Cleanup ALWAYS runs — a JSON-model arducopter without gz sim behind it
# wedges on port 5760 and ignores SIGTERM (hence -9).
cleanup() {
  pkill -9 -f "[a]rducopter" 2>/dev/null || true
  pkill -9 -f "gz si[m]"     2>/dev/null || true
}
trap cleanup EXIT

export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"

echo "=== gz sim (headless server) ==="
cleanup                       # clear any wedged leftovers from prior runs
sleep 1
nohup gz sim -s -r "$HOME/ardupilot_gazebo/worlds/iris_runway.sdf" \
  > ~/sitl-run/gz.log 2>&1 &
sleep 6

echo "=== SITL with JSON backend (talks to gazebo plugin) ==="
mkdir -p "$HOME/sitl-run" && cd "$HOME/sitl-run"
nohup bash -c "tail -f /dev/null | $HOME/ardupilot/build/sitl/bin/arducopter \
  --model JSON --speedup 1 \
  --defaults $HOME/ardupilot/Tools/autotest/default_params/copter.parm,$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm \
  --home -35.363261,149.165230,584,0 -I0" > ~/sitl-run/sitl.log 2>&1 &
sleep 8

echo "=== guardrail mission (pymavlink rail) ==="
cd "/mnt/d/OneDrive/College/S2-TaipeiTech/Lab/VLA Drone"
~/venv-ap/bin/python sitl/run_sitl_demo.py --shield "$SHIELD" $DYN \
  --tag "gazebo_shield_${SHIELD}$( [ -n "$DYN" ] && echo _dynamic )" 2>&1 | grep -v "EOF on TCP"

echo "=== done (cleanup runs via trap) ==="
