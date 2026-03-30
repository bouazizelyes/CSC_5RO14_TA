# MVP Runbook (inspection + capture 3D)

## 1) Build
```bash
cd /home/elyes/CATKIN_WS
catkin_make
source devel/setup.bash
```

## 2) No drone connected now (VRPN-only test)
Install VRPN ROS client once:
```bash
sudo apt install ros-noetic-vrpn-client-ros
```

Start VRPN client:
```bash
cd /home/elyes/CATKIN_WS
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch vrpn_client_ros sample.launch server:=147.250.35.30 port:=3883
```

In another terminal, detect the pose topic and test it:
```bash
cd /home/elyes/CATKIN_WS
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic list | grep '^/vrpn_client_node/.*/pose$'
rostopic echo -n1 /vrpn_client_node/Telloyes/pose
rostopic hz /vrpn_client_node/Telloyes/pose
```

Optional localization-node test without drone video:
```bash
roslaunch tello_driver phase3_localization.launch pose_topic:=/vrpn_client_node/Telloyes/pose
```

## 3) Later, when drone is connected (video + real pose, no fake pose)
Start the dedicated one-command script (recommended):
```bash
cd /home/elyes/CATKIN_WS
bash src/tello_driver/scripts/run_bench_vrpn.sh /home/elyes/CATKIN_WS 147.250.35.30 3883 Telloyes
```

Recommended for better YOLO latency (CPU):
```bash
cd /home/elyes/CATKIN_WS
source .venv/bin/activate
python -m pip install -U ultralytics
# Pre-download model once so runtime does not fail offline.
yolo predict model=yolov8n.pt source='https://ultralytics.com/images/bus.jpg' imgsz=416

SHOW_DEBUG=false \
DETECTOR_BACKEND=yolo \
YOLO_MODEL=/home/elyes/CATKIN_WS/yolov8n.pt \
YOLO_ALLOW_DOWNLOAD=false \
YOLO_DEVICE=cpu \
YOLO_IMGSZ=416 \
YOLO_MAX_DET=20 \
YOLO_PERSON_ONLY=true \
PROCESS_EVERY_N_FRAMES=2 \
bash src/tello_driver/scripts/run_bench_vrpn.sh /home/elyes/CATKIN_WS 147.250.35.30 3883 Telloyes
```

Equivalent manual launch:
```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tello_driver bench_safe_stack.launch use_fake_pose:=false pose_topic:=/vrpn_client_node/Telloyes/pose
```

Check both streams:
```bash
rostopic hz /tello/camera/image_raw
rostopic hz /vrpn_client_node/Telloyes/pose
rostopic hz /tello/object_count
```

## 4) Before first autonomous flight (go/no-go)
Only proceed when all checks pass:
```bash
# 1) Pose is stable and fast.
rostopic hz /vrpn_client_node/Telloyes/pose

# 2) Video is stable.
rostopic hz /tello/camera/image_raw

# 3) Detector runs on YOLO (not fallback).
rosparam get /tello_vision_node/detector_backend

# 3b) Check runtime logs too. If you see "started with backend=hog",
# YOLO failed and the node fell back to HOG.

# 4) Localization-only mode is still active in bench test.
rosparam get /tello_control_node/localization_only
rosparam get /tello_control_node/enable_autonomy
```

For first real autonomy attempt, do not use `bench_safe_stack.launch`.
Use:
```bash
roslaunch tello_driver phase4_autonomy.launch \
	pose_topic:=/vrpn_client_node/Telloyes/pose \
	detector_backend:=yolo \
	yolo_model:=/home/elyes/CATKIN_WS/yolov8n.pt \
	yolo_allow_download:=false \
	yolo_device:=cpu \
	yolo_imgsz:=416 \
	yolo_person_only:=true \
	min_unique_confirmations:=3 \
	process_every_n_frames:=2 \
	show_debug:=false \
	live_view:=true \
	live_view_topic:=/tello/camera/detections \
	safe_wall_distance:=1.2 \
	room_x_min:=-3.2 room_x_max:=2.8 \
	room_y_min:=-2.2 room_y_max:=3.8 \
	room_z_min:=0.8 room_z_max:=1.8 \
	search_speed_x:=0.18 search_speed_y:=0.14 search_speed_z:=0.08 \
	search_lane_spacing_y:=0.8 search_layer_spacing_z:=0.35 \
	search_waypoint_tolerance_xy:=0.20 search_waypoint_tolerance_z:=0.15 \
	land_on_target_count:=false \
	target_count:=999
```

To run coverage + mapping logger (serpentine search + dataset):
```bash
roslaunch tello_driver mapping_mvp.launch \
	pose_topic:=/vrpn_client_node/Telloyes/pose \
	detector_backend:=yolo \
	yolo_model:=/home/elyes/CATKIN_WS/yolov8n.pt \
	yolo_allow_download:=false \
	yolo_device:=cpu \
	yolo_imgsz:=416 \
	yolo_person_only:=true \
	min_unique_confirmations:=3 \
	process_every_n_frames:=2 \
	show_debug:=false \
	safe_wall_distance:=1.2 \
	room_x_min:=-3.2 room_x_max:=2.8 \
	room_y_min:=-2.2 room_y_max:=3.8 \
	room_z_min:=0.8 room_z_max:=1.8 \
	search_speed_x:=0.18 search_speed_y:=0.14 search_speed_z:=0.08 \
	search_lane_spacing_y:=0.8 search_layer_spacing_z:=0.35 \
	search_waypoint_tolerance_xy:=0.20 search_waypoint_tolerance_z:=0.15 \
	search_segment_duration:=5.0 \
	land_on_target_count:=false \
	target_count:=8 \
	capture_fps:=2.0
```

## 5) Emergency stop (if drone connected)
```bash
rostopic pub -1 /tello/land std_msgs/Empty "{}"
```

## 6) Output data
The logger writes to:
- `~/tello_mapping_data/<timestamp>/images/*.jpg`
- `~/tello_mapping_data/<timestamp>/poses.csv`

## 7) Fast 3D reconstruction (recommended offline)
Use COLMAP GUI:
1. New project
2. Images folder = `images/`
3. Run: Feature extraction -> Matching -> Sparse reconstruction
4. Export point cloud screenshots for presentation/report

## 8) What to present in 5 minutes
- VRPN pose topic active at stable rate
- Video and pose alive at the same time (when drone is connected)
- Drone keeps wall safety distance
- Detection/count topic increments
- Captured dataset folder appears
- 3D sparse model screenshot/video
