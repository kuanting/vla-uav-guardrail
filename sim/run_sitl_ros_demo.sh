#!/usr/bin/env bash
# Functional-rail bring-up: ArduPilot SITL + MAVROS 2 + the Prof-Lai Shield stack.
#
# Runs the Phase-1 ROS 2 nodes (safety_shield_node, mavlink_adapter, ros_vla_stub)
# against a real ArduPilot SITL on the same host — the ``dev`` topology from
# docs/03-simulation/topologies.md, minus Docker/Gazebo (SITL alone is enough to
# exercise the MAVLink path the Shield sits on).
#
# Why not colcon: this host is ROS 2 Jazzy and colcon isn't installed. The nodes
# are plain rclpy scripts with __main__ blocks, so we (a) make the workspace
# importable via a user site-packages .pth file — which keeps rclpy's import path
# intact (exporting PYTHONPATH breaks rclpy under Ubuntu's externally-managed
# ROS python) — and (b) run the nodes as python modules with --ros-args instead
# of `ros2 run <pkg>`, since there is no colcon overlay to find executables.
#
# Prereqs (one-time on this lab machine):
#   - ROS 2 Jazzy            /opt/ros/jazzy
#   - mavros                 (ros2 pkg list | grep mavros)
#   - ArduPilot SITL         ~/ardupilot/build/sitl/bin/arducopter
#   - shapely/pydantic/pyyaml in the ROS python (pip install --break-system-packages)
#
# Usage (from WSL):
#   bash sim/run_sitl_ros_demo.sh on      # shield ON  (expect 0 NFZ entry)
#   bash sim/run_sitl_ros_demo.sh off     # shield OFF (bypasses shield node)
set -eo pipefail   # NOTE: -u (nounset) is off — ROS setup.bash references unbound vars

SHIELD="${1:-on}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"   # repo root (kuanting-vla-uav-guardrail/)
FCU_PORT=${FCU_PORT:-5760}                 # SITL MAVLink TCP port

source /opt/ros/jazzy/setup.bash

# --- make the workspace importable without disturbing rclpy ----------------- #
SITE=$(python3 -c "import site; print(site.getusersitepackages())")
mkdir -p "$SITE"
printf '%s\n' \
  "$ROOT/packages/vlaguard-common/src" \
  "$ROOT/packages/policy-dsl/src" \
  "$ROOT/packages/safety-shield/src" \
  "$ROOT" \
  "$ROOT/ros2_ws/src/safety_shield_node" \
  "$ROOT/ros2_ws/src/mavlink_adapter" \
  > "$SITE/vlaguard_workspace.pth"

echo "[setup] workspace importable via $SITE/vlaguard_workspace.pth"
python3 -c "import rclpy; from safety_shield_node.node import ShieldNode; from safety_shield_node.flight_controller import FlightController; from mavlink_adapter.node import MavlinkAdapter; from demo.ros_vla_stub import RosVlaStub; print('[setup] rclpy + all nodes import OK')"

# --- cleanup helper --------------------------------------------------------- #
PIDS=()
cleanup() {
  echo ""
  echo "[cleanup] stopping background processes"
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done
  pkill -f "[a]rducopter" 2>/dev/null || true
  pkill -f "[m]avros_node" 2>/dev/null || true
  pkill -f "[s]afety_shield_node.node" 2>/dev/null || true
  pkill -f "[m]avlink_adapter.node" 2>/dev/null || true
  pkill -f "[r]os_vla_stub" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# --- 1. ArduPilot SITL ------------------------------------------------------ #
echo "[1/5] starting ArduPilot SITL on tcp:$FCU_PORT"
ARDUCOPTER=${ARDUCOPTER:-$HOME/ardupilot/build/sitl/bin/arducopter}
PARM=${PARM:-$HOME/ardupilot/Tools/autotest/default_params/copter.parm}
if [[ ! -x "$ARDUCOPTER" ]]; then
  echo "ERROR: arducopter binary not found at $ARDUCOPTER" >&2
  exit 1
fi
# Run from a scratch dir so eeprom/logs don't pollute the repo. tail -f keeps
# stdin open (SITL exits on EOF in headless runs). TCP 5760 is the default port
# SITL opens; no --serial0 flag needed (this binary rejects it).
SITL_RUN=${SITL_RUN:-$HOME/sitl-run}
mkdir -p "$SITL_RUN"
( cd "$SITL_RUN" && tail -f /dev/null | "$ARDUCOPTER" --model quad --speedup 1 \
    --defaults "$PARM" --home -35.363261,149.165230,584,0 -I0 ) &
PIDS+=($!)
sleep 12   # let SITL boot + EKF warm up

# --- 2. MAVROS 2 ------------------------------------------------------------ #
echo "[2/4] starting mavros (fcu tcp:$FCU_PORT)"
ros2 run mavros mavros_node -r \
  -p fcu_url:="tcp://127.0.0.1:$FCU_PORT" \
  -p gcs_url:="udp://@127.0.0.1:14556" &
PIDS+=($!)
sleep 12   # mavros needs ~8s for the AUTOPILOT_VERSION handshake before it's live

# --- 3. Arm + takeoff (blocking: the copter must be airborne before the VLA  #
#         path drives it) ---------------------------------------------------- #
echo "[3/5] arming + takeoff to cruise altitude (blocking ~30-60 s)"
python3 -m safety_shield_node.flight_controller --ros-args \
  -p cruise_alt_m:=50.0

# --- 4. Shield + adapter + VLA stub (the flying path) ----------------------- #
BUNDLE="$ROOT/bundles/itri-icl-2026-demo-v0.3.0.tar.gz"
if [[ ! -f "$BUNDLE" ]]; then
  echo "[bundle] building $BUNDLE"
  python3 -m policy_dsl ingest "$ROOT/bundles/itri-icl-2026-demo.yaml" -o "$BUNDLE"
fi

echo "[4a/5] starting mavlink_adapter (body -> local-NED)"
python3 -m mavlink_adapter.node --ros-args &
PIDS+=($!)
sleep 2

if [[ "$SHIELD" == "on" ]]; then
  echo "[4b/5] starting safety_shield_node (shield ON)"
  python3 -m safety_shield_node.node --ros-args \
    -p bundle_path:="$BUNDLE" \
    -p audit_path:="$ROOT/episodes/sitl_demo/shield_audit.jsonl" &
  PIDS+=($!)
  sleep 2
else
  echo "[4b/5] shield OFF — running a passthrough so /shield/setpoint mirrors the VLA"
  # minimal passthrough: republish /vla/action_4d as /shield/setpoint (no filtering)
  python3 - "$BUNDLE" <<'PY' &
import sys, rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
rclpy.init()
n = Node("shield_passthrough")
pub = n.create_publisher(Float32MultiArray, "/shield/setpoint", 10)
n.create_subscription(Float32MultiArray, "/vla/action_4d", lambda m: pub.publish(m), 10)
rclpy.spin(n)
PY
  PIDS+=($!)
  sleep 2
fi

echo "[4c/5] starting ros_vla_stub (publishes /vla/action_4d @ 10 Hz)"
python3 -m demo.ros_vla_stub --ros-args &
PIDS+=($!)
sleep 2

# --- 5. Observe ------------------------------------------------------------- #
echo "[5/5] airborne. MAVROS + Shield + VLA topics:"
ros2 topic list | grep -E "action_4d|setpoint|intercept|mavros/setpoint" | sed 's/^/  /'
echo ""
echo "[observe] /shield/intercept events for 45 s (Ctrl+C to end early):"
timeout 45 ros2 topic echo /shield/intercept 2>/dev/null || true

echo "[done] audit log: $ROOT/episodes/sitl_demo/shield_audit.jsonl (if shield was on)"
