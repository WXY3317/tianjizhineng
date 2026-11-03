#!/bin/sh
# Preserve message output before SDK global constructors run.
set -eu
demo_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec 3>&1
export GRIPPER_MESSAGE_FD=3
exec "$demo_dir/build/gripper_demo" "$@" 1>&2
