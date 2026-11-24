#!/usr/bin/env bash
# Shared startup logic for the two fixed gripper zero modes.
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
Usage: ./run_all_first.sh|./run_all_second.sh [--build] [robot_launch_argument:=value ...]
run_all_first.sh  rehomes both grippers and enables them.
run_all_second.sh reuses existing gripper zeros and enables them.
Starts the dual-arm driver, feedback, control, and MQTT scene in the same session.
Robot launch arguments are forwarded to run_real_robot.sh. Scene defaults to
scene_dry_run:=false; pass scene_dry_run:=true for a preview without motion.
HELP
  exit 0
fi

if [[ "${gripper_mode:-}" != first && "${gripper_mode:-}" != second ]]; then
  echo "Start through run_all_first.sh or run_all_second.sh." >&2
  exit 2
fi

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
    echo "Build first: $0 --build" >&2
    exit 1
  fi
done

if [[ "$gripper_mode" == first ]]; then
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
