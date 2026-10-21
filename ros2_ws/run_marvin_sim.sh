#!/usr/bin/env bash
# 旧版 Marvin 仿真启动入口，准备工作区环境并启动 ROS/SDK 控制链路。
set -e

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(dirname -- "$workspace_dir")"
source /opt/ros/humble/setup.bash
cd "$workspace_dir"

if [[ "${1:-}" == "--build" ]]; then
  colcon build --base-paths src "$project_dir/marvin_description" \
    --packages-select marvin_description marvin_sim --symlink-install \
    --cmake-args -DBUILD_TESTING=OFF
  shift
fi
if [[ ! -f install/marvin_sim/share/marvin_sim/package.xml ]]; then
  echo "Build first: $0 --build"
  exit 1
fi
source install/setup.bash
exec ros2 launch marvin_sim simulation.launch.py sdk_root:="$project_dir/tianji_armSDK" "$@"
