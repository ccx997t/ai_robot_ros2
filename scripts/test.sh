#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$root"
[[ -f install/setup.bash ]] || { echo '请先构建工作区' >&2; exit 1; }
set +u
source install/setup.bash
set -u
colcon test --event-handlers console_direct+
colcon test-result --verbose
