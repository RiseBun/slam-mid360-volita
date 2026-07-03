# pose_guard_mapper

ROS2 node for conservative pose arbitration between LiDAR odometry and Vilota camera VIO.

Default behavior:

- Keep using LiDAR odometry from `/odom`.
- Compare LiDAR odometry with `/S1/vio_odom` only when header timestamps are close and poses are sane.
- Switch `/trusted_odom` to camera VIO only after sustained disagreement.
- Accumulate raw `/livox/lidar` points into `/guarded_map` using the trusted pose.

Start after the Livox driver, DP180 ROS bridge, and `adaptive_lio` are running:

```bash
source /opt/ros/humble/setup.bash
source /home/li/livox_ws/install/setup.bash
source /home/li/dp180_ws/install/setup.bash
source /home/li/slam-mid360-volita/install/setup.bash
ros2 launch pose_guard_mapper pose_guard_mapper.launch.py
```

Save the guarded map:

```bash
ros2 service call /pose_guard/save_map std_srvs/srv/Trigger
```
