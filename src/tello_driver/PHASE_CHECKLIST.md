# CSC_5RO14_TA checklist mapped to this workspace

This checklist uses the existing ROS package `tello_driver` in `CATKIN_WS/src/tello_driver` and the new nodes:
- `scripts/tello_vision_node.py`
- `scripts/tello_control_node.py`

## Phase 1: Environment setup and basic control

1. Build workspace
```bash
cd /home/elyes/CATKIN_WS
catkin_make
source devel/setup.bash
```

2. Install dependencies
```bash
cd /home/elyes/CATKIN_WS
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

3. Test 1 (driver connection)
```bash
roslaunch tello_driver tello_node.launch
```

4. Test 2 (telemetry)
```bash
rostopic echo /tello/status
```

5. Test 3 (manual takeoff/land)
```bash
rostopic pub -1 /tello/takeoff std_msgs/Empty "{}"
sleep 3
rostopic pub -1 /tello/land std_msgs/Empty "{}"
```

## Phase 2: CV integration

1. Test 4 (video stream)
```bash
roslaunch tello_driver phase2_cv.launch detector_backend:=hog show_debug:=true
```

2. Test 5 (detection boxes)
```bash
roslaunch tello_driver phase2_cv.launch detector_backend:=yolo
```
If YOLO is used, install:
```bash
pip3 install ultralytics
```

3. Test 6 (tracking and counting)
```bash
rostopic echo /tello/object_count
```

## Phase 3: External localization

1. Launch your camera localization stack first (VRPN/Vicon/OptiTrack).
2. Run localization node:
```bash
roslaunch tello_driver phase3_localization.launch pose_topic:=/vrpn_client_node/tello/pose
```
3. Walk the drone in-hand and watch distance logs in terminal.

## Phase 4: Autonomous control

1. Full system launch:
```bash
roslaunch tello_driver phase4_autonomy.launch pose_topic:=/vrpn_client_node/tello/pose safe_wall_distance:=1.0 target_count:=10
```

2. Test 8 manual override input (optional teleop):
```bash
# Publish user command stream to cmd_vel_user; control node overrides near walls
rostopic pub /tello/cmd_vel_user geometry_msgs/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}' -r 10
```

3. Emergency kill switch:
```bash
rostopic pub -1 /tello/land std_msgs/Empty "{}"
```
