#!/bin/bash
# Opens the ARX X7s controller page.
#   (no option)    sign into the robot and open the controller
#   --deploy       send the controller files to the robot first
#   --simulation   practice offline, no robot needed
# Set ARX_ROBOT=user@address if the robot's address changes.
set -euo pipefail
cd -- "$(dirname -- "$0")"
robot=${ARX_ROBOT:-arx@10.90.134.126}
python=$(command -v python3)
files=(jog.py robot_session.py control_bridge.py can_monitor.py zed_stream.py run-controller.sh run-robot.sh prepare-runtime.sh)

deploy_controller() (
    set -euo pipefail
    staging=$(mktemp -d /tmp/arx-controller-send.XXXXXX)
    trap 'rm -rf -- "$staging"' EXIT
    cp "${files[@]}" "$staging/"
    (cd "$staging" && /usr/bin/shasum -a 256 "${files[@]}" > SHA256SUMS)
    quoted_installer=$("$python" -c 'import shlex, sys; print(shlex.quote(open(sys.argv[1]).read()))' robot-install.sh)
    COPYFILE_DISABLE=1 tar -cf - -C "$staging" "${files[@]}" SHA256SUMS |
        /usr/bin/ssh -T -S "$socket" -o BatchMode=yes -o ControlMaster=no \
            -o ProxyCommand=/usr/bin/false "$robot" "bash -c $quoted_installer"
)

case "${1:-}" in
    --simulation) exec "$python" -B mac_controller.py --simulation --robot "$robot" ;;
    --encoders) printf 'The encoder monitor is now the CAN monitor panel in the controller page.\n' ;;
esac
socket_dir=$(mktemp -d /tmp/arx-mac.XXXXXX)
socket="$socket_dir/ssh"
cleanup() {
    /usr/bin/ssh -S "$socket" -O exit "$robot" >/dev/null 2>&1 || true
    rm -f -- "$socket"
    rmdir "$socket_dir" 2>/dev/null || true
}
trap cleanup EXIT
printf 'Signing into %s…\n' "$robot"
if /usr/bin/ssh -fNT -M -S "$socket" -o StrictHostKeyChecking=yes -o UpdateHostKeys=no \
        -o ConnectTimeout=6 -o NumberOfPasswordPrompts=1 -o ServerAliveInterval=5 \
        -o ServerAliveCountMax=2 -o ClearAllForwardings=yes -o ForwardAgent=no "$robot"; then
    if [[ "${1:-}" == --deploy ]]; then
        deploy_controller
    fi
    "$python" -B mac_controller.py --socket "$socket" --robot "$robot"
else
    if [[ "${1:-}" == --deploy ]]; then
        printf 'Could not sign into the robot. The controller update is still on this Mac.\n' >&2
        exit 1
    fi
    printf 'Could not sign into the robot. Opening the simulator instead.\n' >&2
    "$python" -B mac_controller.py --simulation --robot "$robot"
fi
