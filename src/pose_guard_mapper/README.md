# pose_guard_mapper

ROS2 node for validating LiDAR odometry against Vilota camera VIO and building a
time-consistent fallback map.

The main SLAM launcher does not start this package by default. Passing `--guard`
means a camera is installed, time-synchronized, and extrinsically calibrated.
When this node is running:

- `/trusted_odom` follows LiDAR odometry from `/odom`.
- Camera VIO is compared only when its timestamp and pose are valid.
- Camera VIO takes over after sustained disagreement with LiDAR odometry.
- Every Livox point is transformed at `header.stamp + offset_time`; a scan is
  committed only when trusted poses cover its complete time range.
- The guarded PCD is saved by `/pose_guard/save_map` and again during clean shutdown.

Build and start:

```bash
cd ~/slam-mid360-volita
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
colcon build --packages-select pose_guard_mapper --symlink-install
source install/setup.bash
ros2 launch pose_guard_mapper pose_guard_mapper.launch.py \
  save_map_path:=$HOME/slam_maps/guarded_map.pcd
```

Recorded bags retain historical header timestamps. Use
`require_header_time_near_now:=false` for manual bag playback; `run_slam.sh
--bag ...` sets this automatically.

Do not start the guard while either `camera_child_to_lidar_base_*` or
`base_from_lidar_*` is still a placeholder.

Save manually:

```bash
ros2 service call /pose_guard/save_map std_srvs/srv/Trigger
```
