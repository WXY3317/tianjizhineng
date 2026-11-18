#!/usr/bin/env bash
# Read battery percentages from tianji without joining its DDS network locally.
set -euo pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'HELP'
Usage: ./read_tianji_battery.sh
Reads one /driver/battery/Data message on the SSH host "tianji" and prints
the energy percentage of each battery. Requires SSH access and Python PyYAML.
HELP
  exit 0
fi
if [[ $# -ne 0 ]]; then
  echo "Unexpected argument; use --help for usage" >&2
  exit 2
fi

battery_yaml="$(ssh -o BatchMode=yes -o ConnectTimeout=5 tianji \
  "bash -lc 'source /opt/ros/humble/setup.bash; source /opt/kernelmind/apex/install/setup.bash; export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///etc/apex/cyclonedds.xml; timeout 12 ros2 topic echo /driver/battery/Data --once'")"

printf '%s\n' "$battery_yaml" | python3 -c '
import sys
import yaml

message = next(yaml.safe_load_all(sys.stdin), None)
if not isinstance(message, dict) or not isinstance(message.get("data"), list):
    sys.exit("No battery data received from tianji")
if not message["data"]:
    sys.exit("The battery data list is empty")
for battery in message["data"]:
    if not isinstance(battery, dict) or "id" not in battery or "energy" not in battery:
        sys.exit("Invalid battery data format")
    print("电池 {}：{}%".format(battery["id"], battery["energy"]))
'
