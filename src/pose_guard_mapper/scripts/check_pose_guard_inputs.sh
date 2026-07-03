#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/li/livox_ws/install/setup.bash 2>/dev/null || true
source /home/li/dp180_ws/install/setup.bash 2>/dev/null || true
source /home/li/slam-mid360-volita/install/setup.bash 2>/dev/null || true

set -u

echo "jetson_now_epoch=$(date +%s)"
echo "--- topics ---"
ros2 topic list | grep -E '^(/S1|/livox|/odom|/trusted|/guarded|/pose_guard)' || true

topic_once() {
  local topic="$1"
  if ros2 topic list | grep -qx "${topic}"; then
    timeout 5 ros2 topic echo "${topic}" --once | sed -n '1,10p' || true
  else
    echo "${topic} not listed"
  fi
}

echo "--- /S1/vio_odom stamp ---"
topic_once /S1/vio_odom

echo "--- /odom stamp ---"
topic_once /odom

echo "--- /livox/lidar stamp ---"
topic_once /livox/lidar
