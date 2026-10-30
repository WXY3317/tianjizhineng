#!/usr/bin/env bash
# Start only the control node; reuse the running driver's SDK connection.
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
Usage: ./run_control_only.sh [--build] [--ros-args ...]
Starts only gento_control; does not connect the SDK or start driver/feedback.
First start driver + feedback in another terminal:
  ./run_feedback_only.sh
Then run this script. Do not start a second control node.
--build builds gento_control and its workspace dependencies before starting.
Remaining arguments are forwarded to ros2 run gento_control control.
HELP
  exit 0
fi
export PATH="/usr/bin:$PATH"
ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ ! -f "$ros_setup" ]]; then
  echo "ROS setup not found: $ros_setup" >&2
  exit 1
fi
source "$ros_setup"
cd "$workspace_dir"
if [[ "${1:-}" == "--build" ]]; then
  colcon --log-base log_real build --base-paths src --packages-up-to gento_control \
    --build-base build_real --install-base install_real --symlink-install
  shift
fi
if [[ ! -f install_real/gento_control/share/gento_control/package.xml ]]; then
  echo "Build first: $0 --build" >&2
  exit 1
fi
source install_real/setup.bash
exec ros2 run gento_control control "$@"
