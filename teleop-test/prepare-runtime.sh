#!/usr/bin/env bash
set -euo pipefail
project=$(cd -- "$(dirname -- "$0")" && pwd)
runtime=$(mktemp -d /dev/shm/arx-teleop.XXXXXX)
export TMPDIR="$runtime" PYTHONDONTWRITEBYTECODE=1
python3 -m pip --isolated --disable-pip-version-check --no-cache-dir install \
    --only-binary=:all: --index-url https://pypi.org/simple \
    --target "$runtime/site" -r "$project/requirements.txt"
mkdir "$runtime/project"
cp -a "$project/"{jog.py,can_monitor.py,robot_session.py,control_bridge.py,requirements.txt,run-robot.sh,run-controller.sh,prepare-runtime.sh,README.md,robot-original} "$runtime/project/"
printf '\nRuntime ready at %s\n' "$runtime/project"
