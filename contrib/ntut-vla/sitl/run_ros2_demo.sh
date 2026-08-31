#!/usr/bin/env bash
# Orchestrate the full ROS 2 rail: SITL + mavros + vla node + shield node.
# Usage (WSL):  bash run_ros2_demo.sh [on|off] [--dynamic]
set -e
SHIELD="${1:-on}"
DYN="${2:-}"
DIR="$(cd "$(dirname "$0")" && pwd)"

# Cleanup ALWAYS runs (success, failure, Ctrl+C) — leftover arducopter/mavros
# processes hold port 5760 and silently break every later run.
cleanup() {
  pkill -9 -f "[r]os2_vla_stub_node" 2>/dev/null || true
  pkill -9 -f "mavros_nod[e]"        2>/dev/null || true
  pkill -9 -f "[a]rducopter"         2>/dev/null || true
}
trap cleanup EXIT

source /opt/ros/jazzy/setup.bash
PY=~/venv-ros/bin/python

echo "=== fresh SITL ==="
cleanup                       # clear any wedged leftovers from prior runs
sleep 2
nohup bash "$DIR/start_sitl.sh" > ~/sitl-run/sitl.log 2>&1 &
sleep 8

echo "=== mavros ==="
nohup ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=tcp://127.0.0.1:5760 > ~/sitl-run/mavros.log 2>&1 &
sleep 10

echo "=== vla stub node ==="
pkill -f "[r]os2_vla_stub_node" || true
nohup "$PY" "$DIR/ros2_vla_stub_node.py" > ~/sitl-run/vla.log 2>&1 &
sleep 2

echo "=== shield node (mission) ==="
"$PY" "$DIR/ros2_shield_node.py" --shield "$SHIELD" $DYN

echo "=== done (cleanup runs via trap) ==="
