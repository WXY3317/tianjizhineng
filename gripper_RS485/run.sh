#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec 3>&1
export GRIPPER_MESSAGE_FD=3
exec "$project_dir/build/demo/example/gripper_demo" "$@" 1>&2
