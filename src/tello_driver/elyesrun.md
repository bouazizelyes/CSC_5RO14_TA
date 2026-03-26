What To Test Now (Automated, Safe)

Bench test (recommended): driver + camera + detector + localization node + fake OptiTrack pose.

Do NOT run at home:
- `mapping_mvp.launch`
- `phase4_autonomy.launch`

These can include autonomy/takeoff behavior.

Drone Setup (real hardware)
- Turn drone ON and connect laptop Wi‑Fi to TELLO SSID.
- Keep drone on floor/table, props clear.
- Do not publish `/tello/takeoff`.

One-Time Build
```bash
cd /home/elyes/CATKIN_WS
source /opt/ros/noetic/setup.bash
catkin_make --pkg tello_driver -j$(nproc)
source devel/setup.bash
```

One Command Run (single terminal)
```bash
cd /home/elyes/CATKIN_WS
bash src/tello_driver/scripts/run_bench_safe.sh
```

Equivalent direct command:
```bash
cd /home/elyes/CATKIN_WS
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tello_driver bench_safe_stack.launch
```

What this launches
- `tello_node.launch` (real): talks to drone, publishes camera + telemetry.
- `phase2_cv.launch` with `launch_driver:=false` (real camera input): object detector/counting node.
- `fake_pose_publisher.py` (simulated): publishes fixed pose to `/vrpn_client_node/tello/pose` at 20 Hz.
- `phase3_localization.launch` (real node, safe params): control node in localization-only mode (`enable_autonomy:=false`, `takeoff_on_start:=false`).

What is simulated vs real
- Real: drone video + status topics.
- Simulated: external tracking system (OptiTrack/VRPN pose).
- Result: you validate integration/plumbing without flight commands.

Expected Outputs / Checks

In another terminal:
```bash
cd /home/elyes/CATKIN_WS
source devel/setup.bash
```

1) Camera stream alive
```bash
rostopic hz /tello/image_raw
```
Expected: non-zero frequency.

2) Telemetry alive
```bash
rostopic echo -n1 /tello/status
```
Expected: one `tello_driver/TelloStatus` message with battery/flight-time/etc.

3) Detector publishing count
```bash
rostopic echo /tello/object_count
```
Expected: integer stream (often `0` if no detection, increments when person/object tracked).

4) Control node alive
```bash
rostopic hz /tello/autonomy_state
```
Expected: around control loop rate (default ~10 Hz).

5) Safe control output in localization-only mode
```bash
rostopic echo -n1 /tello/cmd_vel
```
Expected: zero twist (all linear/angular near 0.0).

6) Fake pose stream alive
```bash
rostopic hz /vrpn_client_node/tello/pose
```
Expected: ~20 Hz.

Safety
- Emergency land command (keep ready):
```bash
rostopic pub -1 /tello/land std_msgs/Empty "{}"
```

Stop everything
- Press `Ctrl+C` in the terminal running `run_bench_safe.sh`.

chmod +x src/tello_driver/scripts/fake_pose_publisher.py src/tello_driver/scripts/run_bench_safe.sh