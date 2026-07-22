#!/usr/bin/env bash
# SITL flown A/B — pymavlink direct (bypasses MAVROS, which is flakey on this host).
# Starts ArduPilot SITL, then runs the safety_shield demo against it, shield off
# then on. ArduPilot SITL is RESTARTED between runs (the copter lands and stays
# where it landed otherwise).
#
# Usage (from WSL):  bash sim/run_sitl_flight_ab.sh
set -eo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
source ~/venv-ap/bin/activate

ARDUCOPTER="$HOME/ardupilot/build/sitl/bin/arducopter"
PARM="$HOME/ardupilot/Tools/autotest/default_params/copter.parm"
SITL_RUN="$HOME/sitl-run"
mkdir -p "$SITL_RUN"

start_sitl() {
  echo "[sitl] starting ArduPilot SITL on tcp:5760"
  ( cd "$SITL_RUN" && tail -f /dev/null | "$ARDUCOPTER" --model quad --speedup 1 \
      --defaults "$PARM" --home -35.363261,149.165230,584,0 -I0 ) &
  SITL_PID=$!
  # wait for the TCP listener
  for _ in $(seq 1 30); do
    if python3 -c "import socket; socket.create_connection(('127.0.0.1',5760),1).close()" 2>/dev/null; then
      echo "[sitl] listening"; return
    fi
    sleep 1
  done
  echo "[sitl] ERROR: never opened tcp:5760" >&2; exit 1
}

stop_sitl() {
  echo "[sitl] stopping"
  kill "$SITL_PID" 2>/dev/null || true
  pkill -f "[a]rducopter" 2>/dev/null || true
  sleep 3
}
trap 'stop_sitl' EXIT

# --- Run A: shield OFF (the villain) ---
start_sitl
echo ""; echo "===== RUN A: shield OFF ====="
# shield-off returns non-zero (KPI FAIL by design); don't let set -e abort here.
python3 -m demo.sitl_flight_demo --shield off --tag sitl_shield_off || true
stop_sitl

# --- Run B: shield ON (the hero) ---
start_sitl
echo ""; echo "===== RUN B: shield ON ====="
python3 -m demo.sitl_flight_demo --shield on --tag sitl_shield_on || true
stop_sitl

echo ""
echo "===== done — see episodes/sitl_shield_{off,on}/ ====="
