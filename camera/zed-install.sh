#!/usr/bin/env bash
# Installs the ZED Mini software stack on the ARX robot (Ubuntu 22.04, RTX 40-series).
# Run on the robot as the normal user (not root):  bash ~/zed-setup/zed-install.sh
# Needs the NVIDIA driver already installed and loaded (nvidia-smi must work).
#
# Installs:
#   - CUDA toolkit 12.8 from NVIDIA's apt repo (toolkit only; it does not touch the driver)
#   - ZED SDK 5.6.0 for Ubuntu 22 / CUDA 12.8 / TensorRT 10.9, with the AI module,
#     tools (ZED Explorer, Depth Viewer, ZEDfu mapping) and the pyzed Python API
#   - Python packages for working with maps: numpy, opencv-python, open3d
set -euo pipefail

SDK_VERSION=5.6.0
SDK_FILE="ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v${SDK_VERSION}.zstd.run"
SDK_URL="https://download.stereolabs.com/zedsdk/5.6/cu12/ubuntu22"
SDK_SIZE=1510735845
CUDA_PKG=cuda-toolkit-12-8
CUDA_HOME_DIR=/usr/local/cuda-12.8
SETUP_DIR="$HOME/zed-setup"

say() { printf '\n==> %s\n' "$*"; }
die() { printf '\nERROR: %s\n' "$*" >&2; exit 1; }

[[ $EUID -ne 0 ]] || die "Run this as your normal user, not with sudo. It asks for sudo when needed."
. /etc/os-release
[[ "$VERSION_ID" == "22.04" ]] || die "Expected Ubuntu 22.04, found $PRETTY_NAME."

say "Checking the NVIDIA driver"
if ! command -v nvidia-smi >/dev/null || ! nvidia-smi >/dev/null 2>&1; then
    die "nvidia-smi is not working. Install the NVIDIA driver and reboot first."
fi
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader

mkdir -p "$SETUP_DIR"
cd "$SETUP_DIR"

say "Downloading the ZED SDK installer (1.5 GB) if needed"
if [[ ! -f "$SDK_FILE" || $(stat -c %s "$SDK_FILE") -ne $SDK_SIZE ]]; then
    curl -fL --retry 5 -C - -o "$SDK_FILE" "$SDK_URL"
fi
[[ $(stat -c %s "$SDK_FILE") -eq $SDK_SIZE ]] || die "Installer size is wrong. Delete $SETUP_DIR/$SDK_FILE and retry."
chmod +x "$SDK_FILE"
say "Verifying the installer's built-in checksum"
bash "./$SDK_FILE" --check 2>&1 | tail -c 200

say "Installing apt dependencies (sudo)"
sudo apt-get update
sudo apt-get install -y zstd curl python3-pip python3-dev python3-venv libusb-1.0-0 udev

say "Installing $CUDA_PKG from NVIDIA's repository"
if ! dpkg -s cuda-keyring >/dev/null 2>&1; then
    curl -fL -o cuda-keyring.deb \
        https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
    sudo dpkg -i cuda-keyring.deb
    sudo apt-get update
fi
# Toolkit only. The "cuda" and "cuda-drivers" metapackages would replace the installed driver.
sudo apt-get install -y --no-install-recommends "$CUDA_PKG"

if ! grep -q 'ARX CUDA' "$HOME/.bashrc"; then
    cat >>"$HOME/.bashrc" <<EOF

# ARX CUDA (added by zed-install.sh)
export PATH=$CUDA_HOME_DIR/bin:\$PATH
export LD_LIBRARY_PATH=$CUDA_HOME_DIR/lib64\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}
EOF
fi
export PATH="$CUDA_HOME_DIR/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME_DIR/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
nvcc --version | tail -1

say "Installing the ZED SDK $SDK_VERSION (this also installs pyzed and the camera udev rules)"
"./$SDK_FILE" -- silent

say "Installing Python mapping packages for $USER"
python3 -m pip install --user --upgrade numpy opencv-python open3d

say "Adding $USER to the video group (camera access)"
sudo usermod -aG video "$USER"

say "Done. Log out and back in (or reboot) so the video group and PATH apply,"
say "then run:  python3 ~/zed-setup/zed_check.py"
