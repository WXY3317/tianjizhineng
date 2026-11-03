#!/usr/bin/env bash
# Start only the MQTT vision scene; the robot driver runs separately.
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
Usage: ./run_scene.sh [--build] [mqtt_host:=HOST] [mqtt_port:=PORT] [mqtt_topic:=TOPIC]
Starts the vision MQTT scene only. Start ./run_real_robot.sh separately for robot control.
Defaults: mqtt_host:=127.0.0.1 mqtt_port:=1883 mqtt_topic:=/vision_targe
--build builds the scene package before launch.
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
  colcon --log-base log_real build --base-paths src --packages-select scene \
    --build-base build_real --install-base install_real --symlink-install
  shift
fi
if [[ ! -f install_real/scene/share/scene/package.xml ]]; then
  echo "Build first: $0 --build" >&2
  exit 1
fi
source install_real/setup.bash
exec ros2 launch scene scene.launch.py "$@"
