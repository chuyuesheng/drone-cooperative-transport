#!/usr/bin/env bash
# ============================================================================
# dual_drone_lift — launch a single ArduPilot SITL (webots-python) + UDP bridge
# + MAVProxy for the DJI Mavic 2 PRO in Webots.
#
# Usage (in WSL, after copying this folder into WSL):
#   bash launch_single_drone.sh
#
# Then open dual_drone_lift/worlds/mavic_2_pro.wbt in Webots and press Run.
# In the MAVProxy console that this script opens, send:
#   mode GUIDED
#   arm throttle force
#   takeoff 10
# The drone will climb to 10 m and hover (定点起飞、悬停).
#
# Prerequisites (on this machine, WSL user "luxu"):
#   - ArduPilot built for SITL: ~/ardupilot/build/sitl/bin/arducopter
#   - python3 + pymavlink + MAVProxy (in ~/venv-ardupilot)
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Paths: all overridable via environment variables.
#   ARDUPILOT_ROOT   -> root of the ArduPilot source tree (default ~/ardupilot)
#   PARAMS           -> param defaults file (default: ../params/drone_0.parm)
#   PYTHON / MAVPROXY-> interpreter / script, prefer ~/venv-ardupilot
ARDUPILOT_ROOT=${ARDUPILOT_ROOT:-"$HOME/ardupilot"}
BINARY="$ARDUPILOT_ROOT/build/sitl/bin/arducopter"
BRIDGE="$SCRIPT_DIR/webots_udp_bridge.py"
DEFAULTS="${PARAMS:-$SCRIPT_DIR/../params/drone_0.parm}"

if [ -z "${PYTHON:-}" ]; then
  if [ -x "$HOME/venv-ardupilot/bin/python3" ]; then
    PYTHON="$HOME/venv-ardupilot/bin/python3"
  else
    PYTHON=$(command -v python3 || true)
  fi
fi
if [ -z "${MAVPROXY:-}" ]; then
  if [ -f "$HOME/venv-ardupilot/bin/mavproxy.py" ]; then
    MAVPROXY="$HOME/venv-ardupilot/bin/mavproxy.py"
  else
    MAVPROXY=$(command -v mavproxy.py || true)
  fi
fi

FDM_PORT=${FDM_PORT:-9002}             # Webots <-> SITL motor/FDM port base
SITL_TCP_PORT=${SITL_TCP_PORT:-5760}   # MAVProxy connects here
HOME_STR=${HOME_STR:-"31.2304,121.4737,10,0"}
FRAME=${FRAME:-webots-python}
RUNTIME_DIR="${RUNTIME_DIR:-/tmp/dual_drone_lift}"

# --- sanity checks ----------------------------------------------------------
for c in awk grep; do
  command -v "$c" >/dev/null 2>&1 || { echo "ERROR: missing command: $c"; exit 1; }
done
[ -x "$PYTHON" ]    || { echo "ERROR: python3 not found ($PYTHON); set PYTHON"; exit 1; }
[ -f "$MAVPROXY" ]  || { echo "ERROR: mavproxy.py not found ($MAVPROXY); set MAVPROXY"; exit 1; }
[ -f "$BINARY" ]    || { echo "ERROR: arducopter not found: $BINARY  (set ARDUPILOT_ROOT)"; exit 1; }
[ -f "$BRIDGE" ]    || { echo "ERROR: bridge not found: $BRIDGE"; exit 1; }
[ -f "$DEFAULTS" ]  || { echo "ERROR: params not found: $DEFAULTS"; exit 1; }
"$PYTHON" -c 'import pymavlink' 2>/dev/null || {
  echo "ERROR: pymavlink is unavailable in $PYTHON; set PYTHON to the ArduPilot venv interpreter"
  exit 1
}

# --- IP detection -----------------------------------------------------------
WIN_IP=${WIN_IP:-}
[ -z "$WIN_IP" ] && WIN_IP=$(ip route show default 2>/dev/null | awk '{print $3}' | head -1)
[ -z "$WIN_IP" ] && WIN_IP=$(grep nameserver /etc/resolv.conf 2>/dev/null | awk '{print $2}' | head -1)
[ -n "$WIN_IP" ] || { echo "ERROR: cannot detect Windows IP; set WIN_IP explicitly"; exit 1; }

WSL_IP=${WSL_IP:-}
[ -z "$WSL_IP" ] && WSL_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
[ -n "$WSL_IP" ] || { echo "ERROR: cannot detect WSL IP; set WSL_IP explicitly"; exit 1; }

echo "============================================================"
echo " dual_drone_lift - Mavic2Pro single-drone SITL"
echo " ArduPilot:  $BINARY"
echo " Windows IP: $WIN_IP / WSL IP: $WSL_IP"
echo " FDM port:   $FDM_PORT / SITL TCP: $SITL_TCP_PORT"
echo " Runtime:    $RUNTIME_DIR"
echo "============================================================"

# --- cleanup old processes --------------------------------------------------
pkill -TERM -x arducopter 2>/dev/null || true
pkill -TERM -f webots_udp_bridge.py 2>/dev/null || true
pkill -TERM -f "$MAVPROXY" 2>/dev/null || true
pkill -KILL -x arducopter 2>/dev/null || true
pkill -KILL -f webots_udp_bridge.py 2>/dev/null || true
pkill -KILL -f "$MAVPROXY" 2>/dev/null || true
sleep 1

mkdir -p "$RUNTIME_DIR"
cd "$RUNTIME_DIR"

# --- start UDP bridge (WSL <-> Windows) -------------------------------------
nohup "$PYTHON" "$BRIDGE" --port "$FDM_PORT" --windows-ip "$WIN_IP" --wsl-ip "$WSL_IP" \
  > "$RUNTIME_DIR/bridge.log" 2>&1 < /dev/null &
BRIDGE_PID=$!

# --- start ArduCopter SITL --------------------------------------------------
nohup "$BINARY" \
  --model "${FRAME}:127.0.0.1:${FDM_PORT}" \
  --home "$HOME_STR" \
  --instance 0 \
  --sysid 1 \
  --defaults "$DEFAULTS" \
  --wipe \
  --speedup 1 \
  > "$RUNTIME_DIR/sitl.log" 2>&1 < /dev/null &
SITL_PID=$!
MAVPROXY_PID=""

cleanup() {
  if [ -n "$MAVPROXY_PID" ]; then
    kill -TERM "$MAVPROXY_PID" 2>/dev/null || true
  fi
  kill -TERM "$SITL_PID" "$BRIDGE_PID" 2>/dev/null || true
  wait "$SITL_PID" "$BRIDGE_PID" ${MAVPROXY_PID:+"$MAVPROXY_PID"} 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "    bridge PID=$BRIDGE_PID (log: $RUNTIME_DIR/bridge.log)"
echo "    SITL   PID=$SITL_PID (log: $RUNTIME_DIR/sitl.log)"
echo ""
echo "==> Open dual_drone_lift/worlds/mavic_2_pro.wbt in Webots and press Run."
echo "==> Then send the following in the MAVProxy console below:"
echo "    mode GUIDED"
echo "    arm throttle force"
echo "    takeoff 10"
echo ""
sleep 3

# --- MAVProxy ---------------------------------------------------------------
# Keep the launcher alive when started without a terminal (for example by
# Start-Process from Windows); otherwise MAVProxy sees EOF and immediately
# triggers cleanup of SITL and the UDP bridge.
MAVPROXY_ARGS=(
  "--master=tcp:127.0.0.1:${SITL_TCP_PORT}"
  "--out=udp:${WIN_IP}:14550"
)
if [ -t 0 ]; then
  "$PYTHON" "$MAVPROXY" "${MAVPROXY_ARGS[@]}"
else
  echo "==> No interactive terminal detected; MAVProxy is running headless."
  "$PYTHON" "$MAVPROXY" "${MAVPROXY_ARGS[@]}" --non-interactive &
  MAVPROXY_PID=$!
  wait "$SITL_PID"
fi
