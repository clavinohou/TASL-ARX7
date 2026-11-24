#!/usr/bin/env bash
# Starts the whole-robot control bridge on the robot. One controller at a time.
#   run-controller.sh             fails with an ARX-BUSY line if another session holds the controls
#   run-controller.sh --takeover  stops a previous control bridge first (it releases torque)
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
lock=/dev/shm/arx-teleop-jog.lock
pidfile=/dev/shm/arx-teleop-bridge.pid

holder_is_bridge() {
    local pid
    pid=$(cat "$pidfile" 2>/dev/null || true)
    [[ "$pid" =~ ^[0-9]+$ ]] && [[ -r "/proc/$pid/cmdline" ]] &&
        grep -qa 'control_bridge.py' "/proc/$pid/cmdline" && printf '%s' "$pid"
}

exec 9>>"$lock"
if ! flock -n 9; then
    holder=$(holder_is_bridge || true)
    if [[ "${1:-}" == --takeover && -n "$holder" ]]; then
        kill -TERM "$holder" 2>/dev/null || true
        if ! flock -w 8 9; then
            printf 'ARX-BUSY {"bridge": true, "takeover": "timeout"}\n'
            exit 3
        fi
    elif [[ -n "$holder" ]]; then
        printf 'ARX-BUSY {"bridge": true}\n'
        exit 3
    else
        printf 'ARX-BUSY {"bridge": false}\n'
        exit 3
    fi
fi
printf '%s\n' "$$" > "$pidfile"
sdk="$project/robot-original/myx7s"
export PYTHONPATH="$dependencies"
export LD_LIBRARY_PATH="$sdk/bimanual/api/arx_x7_src:$sdk/lift_controller/api/arx_lift_src:$sdk/lift_controller/api/arx_hardware_interface:/usr/local/lib"
# File descriptor 9 keeps the lock for the life of the controller.
exec python3 -sBu "$project/control_bridge.py" --robot
