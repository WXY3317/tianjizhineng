#!/usr/bin/env bash
# Start both grippers with the launch file's default parameters.
set -e

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PATH="/usr/bin:$PATH"
ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ ! -f "$ros_setup" ]]; then
  echo "ROS setup not found: $ros_setup" >&2
  exit 1
fi
source "$ros_setup"
cd "$workspace_dir"

if [[ "${1:-}" == "--build" ]]; then
  colcon --log-base log build --base-paths src --packages-select gripper \
    --build-base build --install-base install --symlink-install
  shift
fi
if [[ ! -f install/gripper/share/gripper/package.xml ]]; then
  echo "Build first: $0 --build" >&2
  exit 1
fi
source install/setup.bash
exec ros2 launch gripper dual_gripper.launch.py "$@"
