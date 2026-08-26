#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_setup="/opt/ros/humble/setup.bash"
workspace_setup="${project_root}/install/setup.bash"

[[ -f "${ros_setup}" ]] || {
  echo "未找到 ROS 2 Humble 环境：${ros_setup}" >&2
  exit 1
}
[[ -f "${workspace_setup}" ]] || {
  echo "未找到工作区环境：${workspace_setup}" >&2
  echo "请先执行：${project_root}/scripts/build.sh" >&2
  exit 1
}

# ROS setup files may reference unset variables, so temporarily disable nounset.
set +u
source "${ros_setup}"
source "${workspace_setup}"
set -u

cd "${project_root}"
exec ros2 launch ai_robot_bringup navigation.launch.py mode:=sim "$@"
