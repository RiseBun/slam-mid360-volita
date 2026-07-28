#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
default_slam_ws="$(cd "${script_dir}/../../.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
slam_ws="${SLAM_WS_PATH:-$default_slam_ws}"
livox_ws="${LIVOX_WS_PATH:-$HOME/livox_ws}"
dp180_ws="${DP180_WS_PATH:-$HOME/dp180_ws}"

source "/opt/ros/${ros_distro}/setup.bash"
source "${livox_ws}/install/setup.bash" 2>/dev/null || true
source "${dp180_ws}/install/setup.bash" 2>/dev/null || true
source "${slam_ws}/install/setup.bash" 2>/dev/null || true

set -u

echo "host_now_epoch=$(date +%s)"
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
