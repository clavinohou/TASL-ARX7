# Runs on the robot through SSH: bash -c "$(cat robot-install.sh)" < update.tar
# Installs the controller files listed in the archive's SHA256SUMS into the test
# project. Files that differ are copied to .controller-backup/<time>/ first.
set -euo pipefail
project="${ARX_PROJECT:-$HOME/Documents/Teleop Test Enviroment}"
cd "$project"
exec 9>>"${ARX_LOCK:-/dev/shm/arx-teleop-jog.lock}"
if ! flock -n 9; then
    printf 'A controller or jog console is still running on the robot. Disconnect it, then retry.\n' >&2
    exit 1
fi
staging=$(mktemp -d .controller-update.XXXXXX)
cleanup() { rm -rf -- "$staging"; }
trap cleanup EXIT
tar -xf - -C "$staging"
(cd "$staging" && sha256sum --quiet -c SHA256SUMS)
python3 -B - "$staging" <<'ARX_VERIFY'
from datetime import datetime
from pathlib import Path
import shutil
from stat import S_IMODE
import sys

staging = Path(sys.argv[1])
names = [line.split(None, 1)[1].strip().lstrip("*")
         for line in (staging / "SHA256SUMS").read_text().splitlines() if line.strip()]
updates = []
for name in names:
    if "/" in name or name.startswith("."):
        sys.exit(f"Unexpected file name {name!r}. Update stopped.")
    installed, staged = Path(name), staging / name
    source = staged.read_bytes()
    if installed.suffix == ".py":
        # Also proves the file is valid for the robot's Python version.
        compile(source, name, "exec")
    if installed.is_symlink():
        sys.exit(f"{name} on the robot is a symbolic link. Update stopped.")
    current = installed.read_bytes() if installed.is_file() else None
    if current == source:
        continue
    if current is not None:
        mode = S_IMODE(installed.stat().st_mode)
    else:
        mode = 0o755 if name.endswith(".sh") else 0o644
    updates.append((staged, installed, mode, current is not None))

if not updates:
    print("The robot already has these controller files.")
    sys.exit(0)
backup = Path(".controller-backup") / datetime.now().strftime("%Y%m%d-%H%M%S")
for _, installed, _, existed in updates:
    if existed:
        backup.mkdir(parents=True, exist_ok=True)
        shutil.copy2(installed, backup / installed.name)
for staged, installed, mode, _ in updates:
    staged.chmod(mode)
    staged.replace(installed)
print("Updated " + ", ".join(installed.name for _, installed, _, _ in updates) + ".")
if backup.is_dir():
    print(f"Previous versions saved in {Path.cwd() / backup}.")
ARX_VERIFY
sha256sum --quiet -c "$staging/SHA256SUMS"
printf 'Controller update installed and verified.\n'
for interface in can0 can1 can5; do
    if [[ -r "/sys/class/net/$interface/flags" ]] && (( $(cat "/sys/class/net/$interface/flags") & 1 )); then
        printf '%s is up.\n' "$interface"
    else
        printf '%s is down or missing. See Robot-side setup in README.md before enabling.\n' "$interface"
    fi
done
