#!/usr/bin/env bash
# Launch ArduPilot SITL (copter), headless. MAVLink on tcp:127.0.0.1:5760.
# Run in WSL:  bash start_sitl.sh
set -e
BIN="$HOME/ardupilot/build/sitl/bin/arducopter"
PARM="$HOME/ardupilot/Tools/autotest/default_params/copter.parm"
[ -x "$BIN" ] || { echo "SITL binary missing — run setup_sitl.sh first"; exit 1; }

mkdir -p "$HOME/sitl-run"
cd "$HOME/sitl-run"          # eeprom.bin / logs live here, not in the repo
# tail -f /dev/null keeps stdin open forever: SITL's console exits on stdin
# EOF, which kills headless/background runs otherwise (learned the hard way).
tail -f /dev/null | "$BIN" --model quad --speedup 1 --defaults "$PARM" \
     --home -35.363261,149.165230,584,0 -I0
