#!/usr/bin/env bash
# Reuse gripper zero positions, enable both grippers, then start robot control and scene.
gripper_mode=second
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/run_all_common.sh"
