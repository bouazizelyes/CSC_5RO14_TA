#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${1:-/home/elyes/CATKIN_WS}"
POSE_TOPIC="${2:-/vrpn_client_node/Telloyes/pose}"
YOLO_MODEL="${3:-/home/elyes/CATKIN_WS/yolov8n.pt}"

if [ ! -d "$WS_DIR" ]; then
  echo "Workspace not found: $WS_DIR"
  exit 1
fi

cd "$WS_DIR"

set +u
source /opt/ros/noetic/setup.bash
source devel/setup.bash
set -u

if [ ! -f "$YOLO_MODEL" ]; then
  echo "YOLO model file not found: $YOLO_MODEL"
  exit 1
fi

# Stop common conflicting nodes from previous runs.
if command -v rosnode >/dev/null 2>&1; then
  for n in /tello_control_node /tello_vision_node /live_video_view /tello/image_compressed /tello/tello_driver_node; do
    if rosnode list 2>/dev/null | grep -qx "$n"; then
      echo "Stopping existing node: $n"
      rosnode kill "$n" >/dev/null 2>&1 || true
    fi
  done
fi

# Give ROS a moment to release sockets.
sleep 1

if ss -lun 2>/dev/null | grep -q ':9000 '; then
  echo "Port 9000 is still in use. Stop previous tello_driver_node, then retry."
  echo "Hint: pkill -f tello_driver_node"
  exit 1
fi

echo "Launching phase4 autonomy stack..."
exec roslaunch tello_driver phase4_autonomy.launch \
  pose_topic:="$POSE_TOPIC" \
  detector_backend:=yolo \
  yolo_model:="$YOLO_MODEL" \
  yolo_allow_download:=false \
  yolo_device:=cpu \
  yolo_imgsz:=416 \
  yolo_person_only:=true \
  min_unique_confirmations:=3 \
  process_every_n_frames:=2 \
  show_debug:=false \
  live_view:=true \
  live_view_topic:=/tello/camera/detections \
  allow_manual_input:=false \
  safe_wall_distance:=0.50 \
  safe_wall_distance_xy:=0.40 \
  safe_wall_distance_z:=0.35 \
  room_x_min:=1.2 room_x_max:=4.9 \
  room_y_min:=0.6 room_y_max:=5.4 \
  room_z_min:=-0.55 room_z_max:=1.7 \
  search_speed_x:=0.3 search_speed_y:=0.3 search_speed_z:=0.08 \
  search_lane_spacing_y:=0.8 search_layer_spacing_z:=0.35 \
  landing_pre_stop_sec:=1.0 \
  land_on_target_count:=false \
  target_count:=999
