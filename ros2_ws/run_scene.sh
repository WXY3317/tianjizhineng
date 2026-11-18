#!/usr/bin/env bash
# Start only the MQTT vision scene; the robot driver runs separately.
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
Usage: ./run_scene.sh [--build] [mqtt_host:=HOST] [mqtt_port:=PORT] [mqtt_topic:=TOPIC] [dry_run:=true|false] [interval:=SECONDS] [tool_tip_offset_z_m:=METRES] [allow_tool_z_rotation:=true|false] [last_rotation_file:=PATH] [tool_z_angle_file:=PATH]
Starts the vision MQTT scene only. Start ./run_real_robot.sh separately for robot control.
Defaults: mqtt_host:=127.0.0.1 mqtt_port:=1883 mqtt_topic:=vs/workpiece/6d_pose
Default dry_run:=false prints converted targets without sending motion commands.
Default interval:=1.0 waits one second after position1 completes before sending position2.
Default tool_tip_offset_z_m:=0.20 places the tool tip 20 cm along flange local +Z.
Default allow_tool_z_rotation:=true retries equivalent tool Z angles after IK rejection.
Default last_rotation_file:=~/.ros/scene_last_rotation.json records the last completed flange rotation matrix.
Default tool_z_angle_file:=~/.ros/scene_tool_z_angle.json keeps the latest successful nonzero tool Z angle until restored.
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
  colcon --log-base log build --base-paths src --packages-select scene \
    --build-base build --install-base install --symlink-install
  shift
fi
if [[ ! -f install/scene/share/scene/package.xml ]]; then
  echo "Build first: $0 --build" >&2
  exit 1
fi
source install/setup.bash
exec ros2 launch scene scene.launch.py "$@"
