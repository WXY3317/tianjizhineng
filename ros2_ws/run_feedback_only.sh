#!/usr/bin/env bash
# Start driver + feedback with execution enabled; start control separately.
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(dirname -- "$workspace_dir")"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
Usage: ./run_feedback_only.sh [--build] [ip:=CONTROLLER_IPV4] [name:=value ...]
Default IP is configured in src/gento_driver/launch/dual_arm.launch.py (CONTROLLER_IP).
Starts driver + feedback with motion execution enabled, but no control node.
Performs startup recovery and POSITION initialization; does not send a target.
Start ./run_control_only.sh in another terminal to plan and execute motion.
--build builds driver, control and feedback packages before launch.
Options are forwarded to ros2 launch gento_driver dual_arm.launch.py.
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
  colcon --log-base log_real build --base-paths src --packages-up-to gento_driver \
    --build-base build_real --install-base install_real --symlink-install
  shift
fi
sdk_root="$project_dir/tianji-robot-SDK"
for argument in "$@"; do
  case "$argument" in
    sdk_root:=*) sdk_root="${argument#sdk_root:=}" ;;
  esac
done
if [[ ! -f "$sdk_root/PYTHON_SDK/libGentoSDKPY.so" ]]; then
  echo "Missing SDK library: $sdk_root/PYTHON_SDK/libGentoSDKPY.so" >&2
  echo "Provide the hardware-compatible vendor library before starting." >&2
  exit 1
fi
if [[ ! -f install_real/gento_driver/share/gento_driver/package.xml ]]; then
  echo "Build first: $0 --build ip:=CONTROLLER_IPV4" >&2
  exit 1
fi
source install_real/setup.bash
exec ros2 launch gento_driver dual_arm.launch.py "sdk_root:=$sdk_root" "$@" \
  "enable_control:=true" "start_feedback:=true" "start_control:=false"
