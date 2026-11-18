#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec 3>&1
export GRIPPER_MESSAGE_FD=3
exec "$root/bin/gripper_demo" "$@" 1>&2
