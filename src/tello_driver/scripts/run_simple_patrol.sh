#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${1:-/home/elyes/CATKIN_WS}"
VRPN_SERVER="${2:-147.250.35.30}"
VRPN_PORT="${3:-3883}"
TRACKER_NAME="${4:-Telloyes}"

MOVE_SPEED="${MOVE_SPEED:-0.55}"
PAUSE_SEC="${PAUSE_SEC:-1.5}"
BOUNDARY_MARGIN_XY="${BOUNDARY_MARGIN_XY:-0.25}"
POSE_TIMEOUT_SEC="${POSE_TIMEOUT_SEC:-0.25}"
POSE_WAIT_BEFORE_TAKEOFF_SEC="${POSE_WAIT_BEFORE_TAKEOFF_SEC:-8.0}"
TAKEOFF_ON_START="${TAKEOFF_ON_START:-true}"
TAKEOFF_RETRY_PERIOD_SEC="${TAKEOFF_RETRY_PERIOD_SEC:-1.0}"
TAKEOFF_CONFIRM_TIMEOUT_SEC="${TAKEOFF_CONFIRM_TIMEOUT_SEC:-10.0}"
NEAR_WALL_SLOWDOWN_DIST="${NEAR_WALL_SLOWDOWN_DIST:-0.45}"
NEAR_WALL_STOP_DIST="${NEAR_WALL_STOP_DIST:-0.12}"
NEAR_WALL_MIN_SPEED_SCALE="${NEAR_WALL_MIN_SPEED_SCALE:-0.25}"
NEAR_WALL_RETREAT_SPEED="${NEAR_WALL_RETREAT_SPEED:-0.18}"
NEAR_WALL_RETREAT_SEC="${NEAR_WALL_RETREAT_SEC:-0.8}"

if [ ! -d "$WS_DIR" ]; then
  echo "Workspace not found: $WS_DIR"
  echo "Usage: $0 [CATKIN_WS_PATH] [VRPN_SERVER] [VRPN_PORT] [TRACKER_NAME]"
  exit 1
fi

cd "$WS_DIR"

if [ ! -f "devel/setup.bash" ]; then
  echo "Missing devel/setup.bash in $WS_DIR"
  echo "Build first: catkin_make --pkg tello_driver"
  exit 1
fi

set +u
source /opt/ros/noetic/setup.bash
source devel/setup.bash
set -u

if ! rospack find vrpn_client_ros >/dev/null 2>&1; then
  echo "Missing ROS package vrpn_client_ros"
  echo "Install it first: sudo apt install ros-noetic-vrpn-client-ros"
  exit 1
fi

POSE_TOPIC="/vrpn_client_node/${TRACKER_NAME}/pose"
VRPN_LOG="/tmp/vrpn_simple_patrol_$$.log"
STARTED_VRPN=0
VRPN_PID=""

cleanup() {
  if [ "$STARTED_VRPN" -eq 1 ] && [ -n "$VRPN_PID" ]; then
    kill "$VRPN_PID" >/dev/null 2>&1 || true
    wait "$VRPN_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

# Stop conflicting nodes from previous runs.
if command -v rosnode >/dev/null 2>&1; then
  for n in /simple_patrol_node /tello/image_compressed /tello/tello_driver_node; do
    if rosnode list 2>/dev/null | grep -qx "$n"; then
      echo "Stopping existing node: $n"
      rosnode kill "$n" >/dev/null 2>&1 || true
    fi
  done
fi

sleep 1

if ss -lun 2>/dev/null | grep -q ':9000 '; then
  echo "Port 9000 is still in use. Stop previous tello_driver_node, then retry."
  echo "Hint: pkill -f tello_driver_node"
  exit 1
fi

# Start VRPN only when pose topic is absent.
if ! rostopic info "$POSE_TOPIC" >/dev/null 2>&1; then
  echo "Starting VRPN client ($VRPN_SERVER:$VRPN_PORT) for tracker $TRACKER_NAME..."
  roslaunch vrpn_client_ros sample.launch server:="$VRPN_SERVER" port:="$VRPN_PORT" > "$VRPN_LOG" 2>&1 &
  VRPN_PID=$!
  STARTED_VRPN=1

  for _ in $(seq 1 25); do
    if rostopic info "$POSE_TOPIC" >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
fi

if ! rostopic info "$POSE_TOPIC" >/dev/null 2>&1; then
  echo "Pose topic not found: $POSE_TOPIC"
  echo "Known VRPN topics:"
  rostopic list 2>/dev/null | grep '^/vrpn_client_node/' || true
  echo "VRPN log tail:"
  tail -n 40 "$VRPN_LOG" || true
  exit 1
fi

echo "Using pose topic: $POSE_TOPIC"
echo "Launching simple patrol..."
echo "Settings: move_speed=$MOVE_SPEED pause_sec=$PAUSE_SEC boundary_margin_xy=$BOUNDARY_MARGIN_XY near_wall_stop_dist=$NEAR_WALL_STOP_DIST near_wall_retreat_sec=$NEAR_WALL_RETREAT_SEC pose_timeout_sec=$POSE_TIMEOUT_SEC pose_wait_before_takeoff_sec=$POSE_WAIT_BEFORE_TAKEOFF_SEC takeoff_on_start=$TAKEOFF_ON_START takeoff_confirm_timeout_sec=$TAKEOFF_CONFIRM_TIMEOUT_SEC"

exec roslaunch tello_driver simple_patrol.launch \
  pose_topic:="$POSE_TOPIC" \
  control_mode:=world_direct \
  move_speed:="$MOVE_SPEED" \
  pause_sec:="$PAUSE_SEC" \
  boundary_margin_xy:="$BOUNDARY_MARGIN_XY" \
  near_wall_slowdown_dist:="$NEAR_WALL_SLOWDOWN_DIST" \
  near_wall_stop_dist:="$NEAR_WALL_STOP_DIST" \
  near_wall_min_speed_scale:="$NEAR_WALL_MIN_SPEED_SCALE" \
  near_wall_retreat_speed:="$NEAR_WALL_RETREAT_SPEED" \
  near_wall_retreat_sec:="$NEAR_WALL_RETREAT_SEC" \
  live_view:=true \
  require_fresh_pose:=true \
  pose_timeout_sec:="$POSE_TIMEOUT_SEC" \
  pose_wait_before_takeoff_sec:="$POSE_WAIT_BEFORE_TAKEOFF_SEC" \
  takeoff_on_start:="$TAKEOFF_ON_START" \
  takeoff_retry_period_sec:="$TAKEOFF_RETRY_PERIOD_SEC" \
  takeoff_confirm_timeout_sec:="$TAKEOFF_CONFIRM_TIMEOUT_SEC" \
  abort_takeoff_if_no_pose:=true
