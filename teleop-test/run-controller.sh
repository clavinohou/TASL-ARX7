#!/usr/bin/env bash
set -euo pipefail
project=$(cd -- "$(dirname -- "$0")" && pwd)
runtime=$(dirname -- "$project")
if [[ -d "$project/.deps" ]]; then
    dependencies="$project/.deps"
elif [[ "$runtime" == /dev/shm/arx-teleop.* && -d "$runtime/site" ]]; then
    dependencies="$runtime/site"
else
    printf 'The teleop dependencies are missing.\n' >&2
    exit 1
fi
sdk="$project/robot-original/myx7s"
export PYTHONPATH="$dependencies"
export LD_LIBRARY_PATH="$sdk/bimanual/api/arx_x7_src:$sdk/lift_controller/api/arx_lift_src:$sdk/lift_controller/api/arx_hardware_interface:/usr/local/lib"
exec flock -n /dev/shm/arx-teleop-jog.lock python3 -sBu "$project/control_bridge.py" --robot
