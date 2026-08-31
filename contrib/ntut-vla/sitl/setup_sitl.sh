#!/usr/bin/env bash
# ArduPilot SITL setup for WSL Ubuntu 24.04 — run as user nathan.
# Idempotent: safe to re-run; skips finished steps.
set -e
echo "=== [1/4] clone ardupilot ==="
cd ~
if [ ! -d ardupilot ]; then
  git clone --recursive https://github.com/ArduPilot/ardupilot.git
else
  echo "already cloned"
fi

echo "=== [2/4] python venv + deps ==="
if [ ! -d ~/venv-ap ]; then
  python3 -m venv ~/venv-ap
fi
~/venv-ap/bin/pip install -q -U pip
~/venv-ap/bin/pip install -q "empy==3.3.4" pexpect future pymavlink MAVProxy \
  "pydantic>=2,<3" pyyaml shapely
echo "venv ready"

echo "=== [3/4] waf configure ==="
cd ~/ardupilot
source ~/venv-ap/bin/activate
./waf configure --board sitl 2>&1 | tail -3

echo "=== [4/4] build copter (this is the long part) ==="
./waf copter -j"$(nproc)" 2>&1 | tail -5

echo "=== DONE ==="
ls -la build/sitl/bin/arducopter
