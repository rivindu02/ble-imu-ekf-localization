#!/usr/bin/env bash
# Run INSIDE the existing devbox; never creates or modifies containers.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-gui}"
DURATION="${2:-80}"
SCENARIO="${3:-nominal}"
if [[ "$MODE" != gui && "$MODE" != headless ]]; then echo 'Mode must be gui or headless' >&2; exit 2; fi
if [[ "$SCENARIO" != nominal && "$SCENARIO" != beacon_outage ]]; then echo 'Scenario must be nominal or beacon_outage' >&2; exit 2; fi
if [[ ! -r /opt/ros/humble/setup.bash ]]; then echo 'Run inside devbox with ROS 2 Humble installed.' >&2; exit 2; fi
set +u
source /opt/ros/humble/setup.bash
set -u
for program in gzserver python3 timeout; do command -v "$program" >/dev/null || { echo "Missing $program" >&2; exit 2; }; done
if [[ "$MODE" == gui ]]; then
    for program in gzclient rviz2; do command -v "$program" >/dev/null || { echo "Missing $program" >&2; exit 2; }; done
fi
python3 -c 'import rclpy, numpy; from sensor_msgs.msg import Imu; from gazebo_msgs.msg import ModelStates'
if [[ ! -f "$PROJECT_DIR/data/simulated/calibration/fit.json" ]]; then python3 "$PROJECT_DIR/scripts/localization.py" calibrate; fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-75}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11356}"
export GAZEBO_MODEL_DATABASE_URI=""
OUTPUT_DIR="${LOCALIZATION_OUTPUT_DIR:-$PROJECT_DIR/results/localization/gazebo_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUTPUT_DIR"
export ROS_LOG_DIR="$OUTPUT_DIR/ros_logs"
python3 "$PROJECT_DIR/scripts/check_environment.py" --output "$OUTPUT_DIR/environment.json"
# Fail before launching if a server already owns our port.
python3 - <<'PY'
import os,socket,urllib.parse
u=urllib.parse.urlparse(os.environ['GAZEBO_MASTER_URI'])
with socket.socket() as s:
    if s.connect_ex((u.hostname,u.port))==0: raise SystemExit('Gazebo master port already in use; choose another GAZEBO_MASTER_URI')
PY
PIDS=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    sleep 1
    for pid in "${PIDS[@]}"; do kill -KILL "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$PROJECT_DIR/gazebo/localization_lab.world" > "$OUTPUT_DIR/server.log" 2>&1 &
PIDS+=("$!")
if [[ "$MODE" == gui ]]; then
    QT_QPA_PLATFORM=xcb gzclient > "$OUTPUT_DIR/client.log" 2>&1 &
    PIDS+=("$!")
    QT_QPA_PLATFORM=xcb rviz2 -d "$PROJECT_DIR/gazebo/localization.rviz" > "$OUTPUT_DIR/rviz.log" 2>&1 &
    PIDS+=("$!")
fi
# Native IMU plugin must publish; the node watchdog refuses a sensorless success.
echo "Starting $MODE demo: $DURATION simulation seconds ($SCENARIO)."
echo "Progress prints every 5 wall-clock seconds. Results: $OUTPUT_DIR"
echo "The demo stops automatically; slow physics can take longer than $DURATION wall-clock seconds."
python3 "$PROJECT_DIR/gazebo/localization_nodes.py" --duration "$DURATION" --scenario "$SCENARIO" --output-dir "$OUTPUT_DIR" --ros-args -p use_sim_time:=true &
NODE_PID=$!; PIDS+=("$NODE_PID")
wait "$NODE_PID"
python3 "$PROJECT_DIR/scripts/validate_localization_run.py" "$OUTPUT_DIR"
echo "Saved and validated: $OUTPUT_DIR"
