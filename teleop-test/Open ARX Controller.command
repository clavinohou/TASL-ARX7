#!/bin/bash
set -euo pipefail
cd -- "$(dirname -- "$0")"

deploy_controller() (
    set -euo pipefail
    staging=$(mktemp -d /tmp/arx-controller-send.XXXXXX)
    trap 'rm -rf -- "$staging"' EXIT
    files=(jog.py robot_session.py prepare-runtime.sh control_bridge.py)
    cp "${files[@]}" "$staging/"
    (
        cd "$staging"
        /usr/bin/shasum -a 256 "${files[@]}" > SHA256SUMS
    )
    installer=$(cat <<'ARX_INSTALL'
set -euo pipefail
project="$HOME/Documents/Teleop Test Enviroment"
cd "$project"
exec 9>>/dev/shm/arx-teleop-jog.lock
if ! flock -n 9; then
    printf 'The controller is still connected. Support the robot before disconnecting, then retry.\n' >&2
    exit 1
fi
staging=$(mktemp -d .controller-update.XXXXXX)
cleanup() { rm -rf -- "$staging"; }
trap cleanup EXIT
tar -xf - -C "$staging"
(cd "$staging" && sha256sum -c SHA256SUMS)
python3 -B - "$staging" <<'ARX_VERIFY'
from hashlib import sha256
from pathlib import Path
from stat import S_IMODE
import sys

staging = Path(sys.argv[1])
# Install dependencies first so an interrupted transfer can be retried.
previous_versions = {
    "jog.py": {
        "d2b06631573def4ff5d0c05ed844288bec1ed8f4e34d17dc131a017856aa641e",
        "8959ff5120df2ce08e064258639eca057f1442c35b0a67dc104321f40266a05c",
    },
    "robot_session.py": {None},
    "prepare-runtime.sh": {
        "3a562856d15c178f4354ae6a4caadef787ef7d44d7eb251bc729bf7cda9d2f24",
    },
    "control_bridge.py": {
        "f9f7eff323a3d5ac692d61887caa1b4c50af84b07184ca3ed0c0198e6eb9db5c",
        "53a894cdc943b02fff56428839d301defdf7723d61116588862c1ce5b6c59e7f",
    },
}
updates = []
for name, allowed in previous_versions.items():
    installed = Path(name)
    staged = staging / name
    source = staged.read_bytes()
    if installed.suffix == ".py":
        compile(source, name, "exec")
    current = installed.read_bytes() if installed.is_file() else None
    if current == source:
        continue
    digest = sha256(current).hexdigest() if current is not None else None
    if installed.is_symlink() or digest not in allowed:
        sys.exit(f"Unexpected version of {name} on the robot. Update stopped.")
    mode = S_IMODE(installed.stat().st_mode) if current is not None else 0o644
    updates.append((staged, installed, mode))

for staged, installed, mode in updates:
    staged.chmod(mode)
    staged.replace(installed)
ARX_VERIFY
sha256sum -c "$staging/SHA256SUMS"
printf 'Controller update installed and verified.\n'
for interface in can0 can1 can5; do
    if [[ -r "/sys/class/net/$interface/flags" ]] && (( $(cat "/sys/class/net/$interface/flags") & 1 )); then
        ip -brief link show dev "$interface"
    else
        printf '%s is down or missing. See Robot-side setup in README.md before enabling.\n' "$interface"
    fi
done
ARX_INSTALL
    )
    quoted_installer=$(printf '%s' "$installer" | "$python" -c 'import shlex, sys; print(shlex.quote(sys.stdin.read()))')
    COPYFILE_DISABLE=1 tar -cf - -C "$staging" "${files[@]}" SHA256SUMS |
        /usr/bin/ssh -T -S "$socket" -o BatchMode=yes -o ControlMaster=no \
            -o ProxyCommand=/usr/bin/false "$robot" "bash -c $quoted_installer"
)

python=$(command -v python3)
if ! "$python" -B -c 'import tkinter' 2>/dev/null; then
    for candidate in /Library/Frameworks/Python.framework/Versions/*/bin/python3; do
        if [[ -x "$candidate" ]] && "$candidate" -B -c 'import tkinter' 2>/dev/null; then
            python=$candidate
            break
        fi
    done
fi
if [[ "${1:-}" == --simulation ]]; then
    exec "$python" -B mac_controller.py --simulation
fi
socket_dir=$(mktemp -d /tmp/arx-mac.XXXXXX)
socket="$socket_dir/ssh"
robot=arx@10.90.134.126
cleanup() {
    /usr/bin/ssh -S "$socket" -O exit "$robot" >/dev/null 2>&1 || true
    rmdir "$socket_dir" 2>/dev/null || true
}
trap cleanup EXIT
if /usr/bin/ssh -fNT -M -S "$socket" -o StrictHostKeyChecking=yes -o UpdateHostKeys=no \
        -o ConnectTimeout=6 -o NumberOfPasswordPrompts=1 -o ServerAliveInterval=5 \
        -o ServerAliveCountMax=2 -o ClearAllForwardings=yes -o ForwardAgent=no "$robot"; then
    if [[ "${1:-}" == --deploy ]]; then
        deploy_controller
    fi
    if [[ "${1:-}" == --encoders ]]; then
        "$python" -B encoder_view.py --socket "$socket"
    else
        "$python" -B mac_controller.py --socket "$socket"
    fi
else
    if [[ "${1:-}" == --deploy ]]; then
        printf 'Could not sign into the robot. The controller update is still on this Mac.\n' >&2
        exit 1
    fi
    if [[ "${1:-}" == --encoders ]]; then
        printf 'Could not sign into the robot to read encoders.\n' >&2
        exit 1
    fi
    printf 'Could not sign into the robot. Opening the simulator.\n' >&2
    "$python" -B mac_controller.py
fi
