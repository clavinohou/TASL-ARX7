#!/usr/bin/env python3
"""Checks the ZED Mini stack on the robot, layer by layer, and grabs one RGB-D frame.

Run on the robot:  python3 ~/zed-setup/zed_check.py
Saves left.png and depth.png in ~/zed-setup/check/ when the camera works.
"""
from pathlib import Path
import shutil
import subprocess
import sys

OUT = Path.home() / "zed-setup" / "check"
ok = True


def report(name, passed, detail=""):
    global ok
    ok &= passed
    print(f"[{'PASS' if passed else 'FAIL'}] {name}" + (f": {detail}" if detail else ""))


def run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=20).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ""


gpu = run(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"])
report("NVIDIA driver", bool(gpu), gpu or "nvidia-smi not working")

nvcc = shutil.which("nvcc") or "/usr/local/cuda-12.8/bin/nvcc"
cuda = run([nvcc, "--version"]).splitlines()
report("CUDA toolkit", bool(cuda), cuda[-1] if cuda else "nvcc not found")

report("ZED SDK", Path("/usr/local/zed/lib/libsl_zed.so").exists(), "/usr/local/zed")

# USB: Stereolabs vendor 2b03. The video interface only appears on a USB 3 link.
usb3 = False
for dev in Path("/sys/bus/usb/devices").glob("*"):
    try:
        if (dev / "idVendor").read_text().strip() != "2b03":
            continue
        speed = int((dev / "speed").read_text().strip())
        product = (dev / "product").read_text().strip() if (dev / "product").exists() else "?"
        print(f"       USB {dev.name}: {product}, {speed} Mb/s")
        usb3 |= speed >= 5000
    except (OSError, ValueError):
        continue
report("ZED Mini on USB 3", usb3,
       "" if usb3 else "only USB 2 seen; flip the USB-C plug at the camera or use a USB 3 port/cable")

try:
    import pyzed.sl as sl
except ImportError as exc:
    report("pyzed import", False, str(exc))
    sys.exit(1)
report("pyzed import", True, sl.Camera.get_sdk_version())

devices = sl.Camera.get_device_list()
for d in devices:
    print(f"       camera: {d.camera_model} serial {d.serial_number} state {d.camera_state}")
report("Camera detected by SDK", bool(devices))
if not devices:
    sys.exit(1)

init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.HD720
init.depth_mode = sl.DEPTH_MODE.NEURAL
init.coordinate_units = sl.UNIT.METER
cam = sl.Camera()
status = cam.open(init)
report("Camera open (HD720, NEURAL depth)", status == sl.ERROR_CODE.SUCCESS, str(status))
if status != sl.ERROR_CODE.SUCCESS:
    sys.exit(1)

image, depth_view, depth = sl.Mat(), sl.Mat(), sl.Mat()
grabbed = False
for _ in range(30):  # let auto-exposure and depth settle
    grabbed = cam.grab() == sl.ERROR_CODE.SUCCESS
if grabbed:
    cam.retrieve_image(image, sl.VIEW.LEFT)
    cam.retrieve_image(depth_view, sl.VIEW.DEPTH)
    cam.retrieve_measure(depth, sl.MEASURE.DEPTH)
    OUT.mkdir(parents=True, exist_ok=True)
    image.write(str(OUT / "left.png"))
    depth_view.write(str(OUT / "depth.png"))
    h, w = depth.get_height(), depth.get_width()
    err, centre = depth.get_value(w // 2, h // 2)
    print(f"       depth at image centre: {centre:.3f} m; images saved in {OUT}")
report("RGB-D frame grabbed", grabbed)

info = cam.get_camera_information()
print(f"       model {info.camera_model}, serial {info.serial_number}, "
      f"firmware {info.camera_configuration.firmware_version}")
cam.close()
print("\nAll checks passed." if ok else "\nSome checks failed; see above.")
sys.exit(0 if ok else 1)
