# ZED Mini camera on the ARX robot

Goal: build an RGB-D map of the robot's surroundings from the head-mounted ZED Mini, as a base for later research.

## Software stack

| Layer | Version | Notes |
| --- | --- | --- |
| NVIDIA driver | `nvidia-driver-595-open` | Installed by the Linux repair work (nouveau blacklisted, PRIME on-demand). |
| CUDA toolkit | 12.8 (`cuda-toolkit-12-8`) | NVIDIA apt repo, toolkit only so the driver is left alone. `/usr/local/cuda-12.8`. |
| ZED SDK | 5.6.0, Ubuntu 22, CUDA 12.8, TensorRT 10.9 | `/usr/local/zed`. Includes the AI module (NEURAL depth), ZED Explorer, ZED Depth Viewer, ZEDfu (spatial mapping GUI), and `pyzed`. |
| Python | system 3.10, `pip --user` | `numpy`, `opencv-python`, `open3d` for map processing. |

CUDA 12.8 was chosen over the 13.0 build because more research libraries (PyTorch wheels, ROS packages) target it. The 595 driver runs either.

## Install (on the robot)

The driver has to be installed and the robot rebooted first (`nvidia-smi` must work). Then, from a robot shell:

```bash
bash ~/zed-setup/zed-install.sh
```

It asks for the sudo password, downloads the 1.5 GB SDK if it isn't already in `~/zed-setup`, verifies the installer's built-in checksum, and installs everything above. Log out and back in afterwards (video group, PATH).

To copy updated scripts from this folder to the robot:

```bash
scp camera/zed-install.sh camera/zed_check.py arx@10.90.134.126:zed-setup/
```

## Check

```bash
python3 ~/zed-setup/zed_check.py
```

It checks the driver, CUDA, SDK, USB link speed, `pyzed`, and camera detection, then grabs one HD720 frame with NEURAL depth and saves `left.png` and `depth.png` to `~/zed-setup/check/`.

## USB 3 is required

The ZED Mini's video interface only enumerates on a USB 3 link. Over USB 2 only the HID (IMU) interface (`2b03:f681`) appears, and the SDK limits it to VGA at 30 fps at best. If `zed_check.py` reports USB 2, flip the USB-C plug at the camera end (the Stereolabs cable is orientation-sensitive), and make sure the other end is in a blue/SS port on the robot PC.

On 2026-11-24 the camera first showed only `2b03:f681` at USB 2 speed. After moving it to the port at USB address 3-5 / 4-1, swapped with the CAN adapter hub, and flipping the plug, the video interface `2b03:f682` came up at 5000 Mb/s as `/dev/video0` and stayed connected. In that port, one plug orientation gives USB 3 and the other only USB 2.

## Live map in the controller dashboard

The teleop dashboard (`teleop-test`, see its README section "Dashboard") starts `zed_stream.py` on the robot when you press **Start camera**. It shows the RGB and depth feeds and builds a colored 5 cm voxel map in a 3D view. `zed_stream.py` is installed into the teleop project by **Send Controller Update.command**. It runs with the system `python3` and the `pyzed`, NumPy, and OpenCV packages installed by `zed-install.sh`. To try it by hand on the robot (it writes binary to standard output, so discard that):

```bash
python3 ~/"Documents/Teleop Test Enviroment/zed_stream.py" > /dev/null
```

Saved maps go to `~/zed-setup/maps/` as binary PLY files.

