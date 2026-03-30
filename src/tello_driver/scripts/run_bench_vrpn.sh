#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${1:-/home/elyes/CATKIN_WS}"
VRPN_SERVER="${2:-147.250.35.30}"
VRPN_PORT="${3:-3883}"
TRACKER_NAME="${4:-}"
DETECTOR_BACKEND="${DETECTOR_BACKEND:-yolo}"
YOLO_MODEL="${YOLO_MODEL:-yolov8n.pt}"
YOLO_ALLOW_DOWNLOAD="${YOLO_ALLOW_DOWNLOAD:-false}"
YOLO_DEVICE="${YOLO_DEVICE:-auto}"
YOLO_IMGSZ="${YOLO_IMGSZ:-416}"
YOLO_MAX_DET="${YOLO_MAX_DET:-20}"
YOLO_PERSON_ONLY="${YOLO_PERSON_ONLY:-true}"
PROCESS_EVERY_N_FRAMES="${PROCESS_EVERY_N_FRAMES:-2}"
SHOW_DEBUG="${SHOW_DEBUG:-false}"

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

VENV_PY="$WS_DIR/.venv/bin/python"
VENV_ACTIVATE="$WS_DIR/.venv/bin/activate"

if [ ! -x "$VENV_PY" ] || [ ! -f "$VENV_ACTIVATE" ]; then
  echo "Missing workspace virtualenv in $WS_DIR/.venv"
  echo "Create it first, then install deps (including av)."
  echo "Example:"
  echo "  python3 -m venv $WS_DIR/.venv"
  echo "  source $WS_DIR/.venv/bin/activate"
  echo "  pip install av"
  exit 1
fi

set +u
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source "$VENV_ACTIVATE"
set -u

if ! rospack find vrpn_client_ros >/dev/null 2>&1; then
  echo "Missing ROS package vrpn_client_ros"
  echo "Install it first: sudo apt install ros-noetic-vrpn-client-ros"
  exit 1
fi

if ! "$VENV_PY" -c "import av, yaml, rospkg" >/dev/null 2>&1; then
  echo "Missing one or more Python modules required by bench stack: av, yaml, rospkg"
  echo "Install in workspace venv, then retry:"
  echo "  source $VENV_ACTIVATE"
  echo "  pip install av PyYAML rospkg catkin_pkg empy"
  exit 1
fi

VRPN_LOG="/tmp/vrpn_client_$$.log"
STARTED_VRPN=0
VRPN_PID=""

cleanup() {
  if [ "$STARTED_VRPN" -eq 1 ] && [ -n "$VRPN_PID" ]; then
    kill "$VRPN_PID" >/dev/null 2>&1 || true
    wait "$VRPN_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

POSE_TOPIC=""
if [ -n "$TRACKER_NAME" ]; then
  POSE_TOPIC="/vrpn_client_node/${TRACKER_NAME}/pose"
fi

# Reuse existing VRPN stream if already available.
if [ -n "$POSE_TOPIC" ] && rostopic info "$POSE_TOPIC" >/dev/null 2>&1; then
  echo "Using existing VRPN pose topic: $POSE_TOPIC"
elif [ -z "$POSE_TOPIC" ]; then
  POSE_TOPIC="$(rostopic list 2>/dev/null | grep -E '^/vrpn_client_node/.*/pose$' | head -n1 || true)"
  if [ -n "$POSE_TOPIC" ]; then
    echo "Using existing VRPN pose topic: $POSE_TOPIC"
  fi
fi

if [ -z "$POSE_TOPIC" ] || ! rostopic info "$POSE_TOPIC" >/dev/null 2>&1; then
  echo "Starting VRPN client ($VRPN_SERVER:$VRPN_PORT)..."
  roslaunch vrpn_client_ros sample.launch server:="$VRPN_SERVER" port:="$VRPN_PORT" > "$VRPN_LOG" 2>&1 &
  VRPN_PID=$!
  STARTED_VRPN=1

  for _ in $(seq 1 30); do
    if [ -n "$TRACKER_NAME" ]; then
      if rostopic info "$POSE_TOPIC" >/dev/null 2>&1; then
        break
      fi
    else
      POSE_TOPIC="$(rostopic list 2>/dev/null | grep -E '^/vrpn_client_node/.*/pose$' | head -n1 || true)"
      if [ -n "$POSE_TOPIC" ]; then
        break
      fi
    fi
    sleep 1
  done
fi

if [ -n "$TRACKER_NAME" ] && ! rostopic info "$POSE_TOPIC" >/dev/null 2>&1; then
  echo "Requested tracker '$TRACKER_NAME' was not found. Trying auto-discovery..."
  for _ in $(seq 1 5); do
    POSE_TOPIC="$(rostopic list 2>/dev/null | grep -E '^/vrpn_client_node/.*/pose$' | head -n1 || true)"
    if [ -n "$POSE_TOPIC" ]; then
      break
    fi
    sleep 1
  done
fi

if [ -z "$POSE_TOPIC" ] || ! rostopic info "$POSE_TOPIC" >/dev/null 2>&1; then
  echo "Could not find a VRPN pose topic."
  echo "If you know the rigid body name, rerun with TRACKER_NAME:"
  echo "  $0 $WS_DIR $VRPN_SERVER $VRPN_PORT <TRACKER_NAME>"
  echo "Visible VRPN topics:"
  rostopic list 2>/dev/null | grep '^/vrpn_client_node/' || true
  echo "Recent VRPN log:"
  tail -n 40 "$VRPN_LOG" || true
  exit 1
fi

echo "Using pose topic: $POSE_TOPIC"
echo "Starting bench stack with real VRPN pose (fake pose disabled)..."
echo "Press Ctrl+C to stop all nodes."
echo "Vision settings: backend=$DETECTOR_BACKEND device=$YOLO_DEVICE imgsz=$YOLO_IMGSZ person_only=$YOLO_PERSON_ONLY stride=$PROCESS_EVERY_N_FRAMES debug=$SHOW_DEBUG"

roslaunch tello_driver bench_safe_stack.launch \
  use_fake_pose:=false \
  pose_topic:="$POSE_TOPIC" \
  detector_backend:="$DETECTOR_BACKEND" \
  yolo_model:="$YOLO_MODEL" \
  yolo_allow_download:="$YOLO_ALLOW_DOWNLOAD" \
  yolo_device:="$YOLO_DEVICE" \
  yolo_imgsz:="$YOLO_IMGSZ" \
  yolo_max_det:="$YOLO_MAX_DET" \
  yolo_person_only:="$YOLO_PERSON_ONLY" \
  process_every_n_frames:="$PROCESS_EVERY_N_FRAMES" \
  show_debug:="$SHOW_DEBUG"
