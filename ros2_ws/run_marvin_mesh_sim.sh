#!/usr/bin/env bash
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(dirname -- "$workspace_dir")"
source /opt/ros/humble/setup.bash
cd "$workspace_dir"
if [[ "${1:-}" == "--build" ]]; then
  PYTHONPATH="$workspace_dir/src/marvin_sim:$workspace_dir/src/marvin_mesh_sim:${PYTHONPATH:-}" \
    python3 -c 'from marvin_mesh_sim.generate import main; main()' \
    --source-root "$project_dir/mjcf" --destination "$workspace_dir/src/marvin_mesh_sim" \
    --sdk-root "$project_dir/tianji_armSDK"
  colcon build --base-paths src "$project_dir/marvin_description" \
    --packages-select marvin_description marvin_sim marvin_mesh_sim --symlink-install \
    --cmake-args -DBUILD_TESTING=OFF
  shift
fi
if [[ ! -f install/marvin_mesh_sim/share/marvin_mesh_sim/package.xml ]]; then
  echo "Build first: $0 --build"
  exit 1
fi
source install/setup.bash

# A closed RViz window does not terminate the SDK controller. Reuse that
# controller on subsequent starts instead of binding its fixed UDP ports again.
workspace_id="$(printf '%s' "$workspace_dir" | sha256sum)"
workspace_id="${workspace_id:0:16}"
runtime_dir="${XDG_RUNTIME_DIR:-/tmp}"
exec 9>"$runtime_dir/marvin-mesh-sim-${UID}-${workspace_id}-${ROS_DOMAIN_ID:-0}.lock"
if ! flock -n 9; then
  echo "复用已运行的仿真后台，重新打开界面。"
  want_rviz=true
  want_panel=true
  for argument in "$@"; do
    case "$argument" in
      rviz:=*) want_rviz="${argument#rviz:=}" ;;
      panel:=*) want_panel="${argument#panel:=}" ;;
    esac
  done
  if [[ "${want_rviz,,}" =~ ^(false|0)$ && "${want_panel,,}" =~ ^(false|0)$ ]]; then
    exit 0
  fi
  exec ros2 launch marvin_mesh_sim view.launch.py sdk_root:="$project_dir/tianji_armSDK" "$@"
fi

sim_ip=""
for argument in "$@"; do
  case "$argument" in sim_ip:=*) sim_ip="${argument#sim_ip:=}" ;; esac
done
python3 - "$sim_ip" <<'PY'
import socket
import sys
from marvin_sim.local_address import local_address

address = local_address(sys.argv[1])
for host, port in ((address, 4729), ('0.0.0.0', 4730), (address, 4741)):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind((host, port))
        except OSError as exc:
            print(f'无法启动：仿真端口 {host}:{port} 已被其他程序占用。', file=sys.stderr)
            print('请结束旧仿真的启动终端后重试；仅关闭 RViz 不会停止后台。', file=sys.stderr)
            sys.exit(1)
PY
exec ros2 launch marvin_mesh_sim simulation.launch.py sdk_root:="$project_dir/tianji_armSDK" "$@"
