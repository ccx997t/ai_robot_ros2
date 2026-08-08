#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
printf '=== S1-M0 环境自检 ===\n工作区: %s\n' "$root"
. /etc/os-release
printf '系统: %s\n内核: %s\n' "$PRETTY_NAME" "$(uname -r)"
df -h "$root" | awk 'NR==2 {print "磁盘可用: " $4}'
if [[ ! -f /opt/ros/humble/setup.bash ]]; then echo '未找到 ROS 2 Humble' >&2; exit 1; fi
set +u
source /opt/ros/humble/setup.bash
set -u
printf 'ROS: %s\nros2: %s\ncolcon: %s\n' "$ROS_DISTRO" "$(command -v ros2)" "$(command -v colcon)"
for tool in rviz2 gz gazebo; do command -v "$tool" >/dev/null && echo "$tool: 可用" || echo "$tool: 未检测到"; done
