# MVP Runbook (inspection + capture 3D)

## 1) Build
```bash
cd /home/elyes/CATKIN_WS
catkin_make
source devel/setup.bash
```

## 2) Start localization first (VRPN/Optitrack)
Make sure pose topic exists:
```bash
rostopic echo -n1 /vrpn_client_node/tello/pose
```

## 3) Start full mission + logger
```bash
roslaunch tello_driver mapping_mvp.launch pose_topic:=/vrpn_client_node/tello/pose safe_wall_distance:=1.0 target_count:=8 capture_fps:=2.0
```

## 4) Emergency stop
```bash
rostopic pub -1 /tello/land std_msgs/Empty "{}"
```

## 5) Output data
The logger writes to:
- `~/tello_mapping_data/<timestamp>/images/*.jpg`
- `~/tello_mapping_data/<timestamp>/poses.csv`

## 6) Fast 3D reconstruction (recommended offline)
Use COLMAP GUI:
1. New project
2. Images folder = `images/`
3. Run: Feature extraction -> Matching -> Sparse reconstruction
4. Export point cloud screenshots for presentation/report

## 7) What to present in 5 minutes
- Launch command works in one line
- Drone keeps wall safety distance
- Detection/count topic increments
- Captured dataset folder appears
- 3D sparse model screenshot/video
