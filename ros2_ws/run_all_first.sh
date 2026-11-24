#!/usr/bin/env bash
# Rehome and enable both grippers, then start robot control and scene.
gripper_mode=first
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/run_all_common.sh"
