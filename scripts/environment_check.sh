#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
printf '=== S1-M0 环境自检 ===\n工作区: %s\n' "$root"
. /etc/os-release
printf '系统: %s\n内核: %s\n' "$PRETTY_NAME" "$(uname -r)"
if [[ "$ID" != "ubuntu" || "$VERSION_ID" != "22.04" ]]; then
  echo '系统不符合 Ubuntu 22.04 基线' >&2
  exit 1
fi
df -h "$root" | awk 'NR==2 {print "磁盘可用: " $4}'
if [[ ! -f /opt/ros/humble/setup.bash ]]; then echo '未找到 ROS 2 Humble' >&2; exit 1; fi
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ "$ROS_DISTRO" != "humble" ]]; then
  printf 'ROS 发行版不符合 Humble 基线: %s\n' "$ROS_DISTRO" >&2
  exit 1
fi

for tool in ros2 colcon rosdep rviz2 ign git; do
  printf '%s: %s\n' "$tool" "$(command -v "$tool")"
done

printf 'Gazebo Fortress: %s\n' "$(ign gazebo --versions)"
for package in ros_gz_sim ros_gz_bridge; do
  printf '%s: %s\n' "$package" "$(ros2 pkg prefix "$package")"
done

if command -v lspci >/dev/null 2>&1; then
  graphics="$(lspci | grep -Ei 'vga|3d|display' || true)"
  printf '图形设备: %s\n' "${graphics:-未检测到独立图形设备}"
fi
printf '代理: http_proxy=%s https_proxy=%s\n' "${http_proxy:-未设置}" "${https_proxy:-未设置}"
