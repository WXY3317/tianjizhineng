#!/usr/bin/env bash
# Start the grippers, real robot control, and MQTT scene from one terminal.
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
Usage: ./run_all.sh first|second [--build] [robot_launch_argument:=value ...]
first  rehomes both grippers and enables them.
second reuses existing gripper zeros and enables them.
Starts the dual-arm driver, feedback, control, and MQTT scene in the same session.
Robot launch arguments are forwarded to run_real_robot.sh. Scene defaults to
scene_dry_run:=false; pass scene_dry_run:=true for a preview without motion.
HELP
  exit 0
fi

mode="${1:-}"
if [[ "$mode" != first && "$mode" != second ]]; then
  echo "Expected first or second. Run $0 --help for usage." >&2
  exit 2
fi
shift

if [[ "${1:-}" == "--build" ]]; then
  export PATH="/usr/bin:$PATH"
  ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  if [[ ! -f "$ros_setup" ]]; then
    echo "ROS setup not found: $ros_setup" >&2
    exit 1
  fi
  source "$ros_setup"
  colcon --log-base log build --base-paths src --packages-up-to gento_driver scene gripper \
    --build-base build --install-base install --symlink-install
  shift
fi

for package in gento_driver scene gripper; do
  if [[ ! -f "install/$package/share/$package/package.xml" ]]; then
    echo "Build first: $0 $mode --build" >&2
    exit 1
  fi
done

if [[ "$mode" == first ]]; then
  ./run_gripper_frist.sh &
else
  ./run_gripper_second.sh &
fi
gripper_pid=$!

cleanup() {
  trap - EXIT INT TERM
  kill -INT "$robot_pid" "$gripper_pid" 2>/dev/null || true
  wait "$robot_pid" "$gripper_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

./run_real_robot.sh start_scene:=true scene_dry_run:=false "$@" &
robot_pid=$!

set +e
wait -n "$gripper_pid" "$robot_pid"
result=$?
set -e
exit "$result"
