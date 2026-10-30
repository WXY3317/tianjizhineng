#!/usr/bin/env bash
# Real dual-arm SDK bridge only; never starts the local simulator.
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(dirname -- "$workspace_dir")"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
Usage: ./run_real_robot.sh [--build] [ip:=CONTROLLER_IPV4] [name:=value ...]
Default IP is configured in src/gento_driver/launch/dual_arm.launch.py (CONTROLLER_IP).
Connects and publishes feedback. Valid targets execute directly.
Use start_control:=false to start driver + feedback with control enabled,
then start ./run_control_only.sh in another terminal.
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
exec ros2 launch gento_driver dual_arm.launch.py "sdk_root:=$sdk_root" "$@" "enable_control:=true"
