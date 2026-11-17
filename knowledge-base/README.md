# TASL ARX X7s robot

## Access

Verified after the November 12, 2026 reboot.

| Access | Address | Observed status |
| --- | --- | --- |
| SSH | `ssh arx@10.90.134.126` or `ssh arx@arx.local` | Signed in over the IPv4 address. `arx.local` resolved to the same address. |
| Remote Desktop | `10.90.134.126:3389` in Windows App | Port reachable from the Mac. GNOME Remote Desktop was active, with an autologged-in Wayland session and `extend` mode. |

The robot was on eduroam. Linux reported all physical display connectors as disconnected at the time of the check. A fresh Windows App login and picture were still unverified. The SSH login came from the earlier **Set up local SSH and desktop access** chat. The notes record the connection details only.

### Display freeze follow-up

On November 12, GNOME removed its RDP virtual monitor at 5:42:05 p.m. EST. A Nouveau GPU message followed a second later. In a later kernel stack capture, a Nouveau hotplug worker was blocked in `nouveau_dp_detect` and GNOME Shell was waiting in `drm_modeset_lock` through `nouveau_drm_ioctl`. This explains the subsequent RDP setup timeouts. The transition is a likely trigger, pending a controlled test. SSH still worked.

The ASUS NUC14SRK-B routes HDMI and full-size DisplayPort through NVIDIA. Rear Thunderbolt USB-C uses Intel graphics. The robot registered a USB-C partner after reconnection, while both Intel display connectors still reported `disconnected`. Check the monitor's selected input and a known-good video-capable cable or adapter. Ubuntu's graphical update remained paused during `libpam-cap` configuration. Finish that update if its prompt becomes reachable, then reboot cleanly with the monitor connected to reset the graphics driver. Ubuntu identified `nvidia-driver-595-open` as its recommended candidate for the NVIDIA card. Validate the supported driver after resolving the package transaction and storage fault, then test a physical display and RDP reconnect. [ASUS port routing](https://dlcdnets.asus.com/pub/ASUS/NUC/NUC_14_Performance/E24132_ROG_NUC_NUC14SRK_Minipc_TPS_EM_V3_WEB.pdf?model=ROG+NUC) · [Ubuntu driver guidance](https://documentation.ubuntu.com/inference-snaps/how-to/install-drivers/)

After the restart, Intel detected the Sculptor monitor and GNOME reported an active 1920×1080 at 60 Hz mode. SSH worked and the RDP port was reachable. The new boot still logged a Nouveau warning and unrecovered NVMe reads. Ubuntu marked `libpam-cap` as requiring reinstallation after the interrupted update.

## Installed teleop

The lab's Python workspace is `/home/arx/ws/myx7s` on Ubuntu 22.04.5 with Python 3.10.12. It uses the local ARX Python SDK and SocketCAN. `/home/arx/X7s` contains the vendor SDK, including separate ROS examples. [ARX X7s vendor repository](https://github.com/ARXroboticsX/X7s)

| Robot file | Role |
| --- | --- |
| `~/ws/myx7s/vr_fanout.py` | Receives headset traffic on UDP `7000` and forwards it locally to `7001` and `7002`. |
| `~/ws/myx7s/vr_base.py` | Receives on `7001` and commands the base, lift, and head through `can5`. |
| `~/ws/myx7s/vr_dual_arm.py` | Receives on `7002` and controls the left and right arms through `can0` and `can1`. |
| `~/ws/myx7s/start_vr_system.sh` and `run_vr_system.py` | Installed launcher with `fanout`, `base`, `dual`, and `all` modes. |
| `~/ws/save/udp.cs` | Unity XR sender source. The configured destination must be the robot's IPv4 address and port `7000`. The source defaults to loopback. The deployed headset app and its destination still need checking. |

The Python and shell scripts passed syntax checks. The receiver forwarded a 98-byte frame to two isolated loopback sockets with zero errors. Both ARX extensions match the installed Python ABI and their shared libraries resolve after sourcing the two setup scripts. This check covered the receiver and dependencies. Live headset reception and motor control remain untested. The lab README lists 73 bytes, while the Python format and Unity sender both use 98 bytes.

The copied `libarx_x7_src.so` contains ARX forward and inverse kinematics functions and Orocos KDL solvers. `bimanual/script/single_arm.py` loads a URDF for the chosen arm and exposes `get_ee_pose()` and `set_ee_pose()` for Cartesian feedback and targets. `set_catch_pos()` is the gripper API already used by the VR controller. A Mac keyboard client can send base, arm jog, pose hold, and gripper commands over SSH to a controller running on the robot. Continuous driving needs a robot-side command timeout and a tested brake on input loss. [Orocos KDL documentation](https://docs.orocos.org/kdl/overview.html)

## What to resolve before motion

1. **Preserve the lab files and address the SSD read errors.** NumPy 2.2.6 imported, while importing `scipy.spatial.transform.Rotation` from SciPy 1.15.3 crashed with `SIGBUS`. Reading the installed `scipy/linalg/_flapack.cpython-310-x86_64-linux-gnu.so` returned `EIO`. The kernel logged `critical medium error` on `/dev/nvme0n1`, the drive hosting `/`. Back up the robot-specific code and configuration before reinstalling dependencies or troubleshooting the drive further. Recheck the SciPy import after recovery.
2. **Verify the actual headset stream with the motor controllers stopped.** On a private robot network, start only `python3 -u vr_fanout.py` from `~/ws/myx7s`, start the headset app, and check that the `RX` count rises. The current receiver accepts unauthenticated UDP on every IPv4 interface. Use a private control network or restrict the receiver to the trusted sender before enabling motion.
3. **Check loss-of-input and shutdown.** `vr_base.py` sends head and lift targets before receiving VR data and can retain its last drive command when packets stop arriving. Add and verify a stale-input stop before driving the base. `vr_dual_arm.py` initializes both arms before opening its UDP socket. The bundled launcher captures child output without reading it and stops children with `SIGTERM`, while the base script handles `SIGINT` for orderly shutdown. Use individual foreground processes for the first supervised tests, with someone at the robot and the physical stop available.

## CAN and startup reference

Three CANable2 adapters and the robot's serial-number rules were present after reboot. The matching Linux CAN interfaces had yet to be created.

| Subsystem | Stable adapter path | Interface |
| --- | --- | --- |
| Left arm | `/dev/arxcan0` | `can0` |
| Right arm | `/dev/arxcan1` | `can1` |
| Base, lift, head | `/dev/arxcan5` | `can5` |

The mapping is in `/etc/udev/rules.d/arx_can.rules`. The installed `~/X7s/ARX_CAN/arx_can/arx_can*.sh` scripts use `slcand -o -f -s8`. Their retry loops kill all `slcand` processes, so use one attachment per required adapter during supervised bring-up. For example, with `can0` absent, the left-arm adapter sequence would be as follows. [CANable SocketCAN setup](https://canable.io/getting-started.html)

```bash
sudo modprobe slcan
sudo slcand -o -f -s8 /dev/arxcan0 can0
sudo ip link set dev can0 up
ip -brief link show dev can0
```

From a robot terminal, set the library paths in the same shell that will run the controllers. Each setup script uses the current directory to find the libraries.

```bash
cd ~/ws/myx7s
source ./setup_arm.sh
source ./setup_base.sh
```

The intended startup order is the UDP receiver, then the controller for the subsystem under test. The controller entry points are `python3 -u vr_dual_arm.py` and `python3 -u vr_base.py`. The bundled full-system entry point is `bash start_vr_system.sh all`. The inspection left the CAN interfaces and motor controllers untouched.
