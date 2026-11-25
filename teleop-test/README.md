# ARX X7s teleop test

`robot-original/myx7s` is the lab control stack copied from the robot. It includes the Linux Python 3.10 arm and lift SDK, robot models, VR receiver, and original launcher. The CAN mapping and older examples are under `robot-original`. The Mac has a separate backup of the selected control files and CAN rule.

For tomorrow's supervised hardware run, follow [LAB-TEST.md](LAB-TEST.md).

## How the controller fits together

```
Browser page ──HTTP on 127.0.0.1──► mac_controller.py ──SSH──► run-controller.sh ─► control_bridge.py ─► robot_session.py ─► ARX SDK / CAN
(controller_ui.html)                (input rules, link)        (lock, takeover)      (leases, timeouts)    (startup, chassis, park)
```

- **The page** draws the robot state and reports which keys are held and whether it has focus. It has no safety logic of its own.
- **`mac_controller.py`** owns the SSH link and the input rules: keys act only while the page has focus (and, if you turn it on, while Space is held), keys already held when control becomes possible are ignored until pressed again, focus loss stops the robot, and stale page input counts as released. It serves the page only on `127.0.0.1` and every request needs a random per-launch token.
- **`control_bridge.py`** runs on the robot. Every status line carries a lease, and the Mac's input counts only as a reply to a status sent within the last 0.30 s.
- **`robot_session.py`** holds the startup sequence, chassis control, park, mapping mode, fault and recovery logic, and a simulator used on the Mac and in the tests.
- **`zed_stream.py`** runs on the robot, separately from the control bridge, in its own SSH channel. It streams the ZED Mini's color and depth images, the camera pose, and a growing voxel map. **`camera_feed.py`** on the Mac receives them, or simulates them offline.

## Dashboard

The page is split in two:

- **Left panel: teleop.** Phase and buttons, the input indicator and Space setting, chassis command and wheel feedback, arms, torso/lift/head, events, and the CAN monitor.
- **The rest: the 3D world.** It shows the voxel map the robot is building and a model of the robot placed at the camera's tracked pose, with its driven path and the camera's view cone.
  - **Chase** follows behind the robot like a third-person game camera. **Top** (the default) looks straight down and keeps the map heading fixed as the robot turns. **Orbit** circles the robot.
  - Drag to look around, scroll to zoom, and double-click to recenter.
- **Top-right corner:** the live RGB feed. **Bottom-right corner:** the live depth view, colored from near (red) to 4 m (blue); black means no depth.
- **Live map panel (top left of the 3D view):** Start/Stop camera, **Source** (Voxel/ZED Fusion), **Build** (Auto/On/Off), Reset, and Save .ply.
- **Voxel** uses our reconstruction. **ZED Fusion** uses the SDK's fused point cloud, with a sampled preview in the browser and full cloud export via **Save .ply**. Switching sources clears the current map. Save it first if you want to compare both runs.

The camera runs independently of the robot controls: it can stream before Connect and keeps running after Disconnect until you stop it.

### Mapping mode

**Mapping mode** (from Ready or Paused) folds both arms to home and returns the torso upright, the same arm and torso motion as Park. The lift stays where it is, so the camera keeps its height. Once the phase reads **Mapping mode**:

- Use **W/A/S/D/←/→** to drive as usual.
- **R/F** (lift, below the lift ceiling) and **1/2** (head pitch) still work.
- Arm, gripper, and torso keys are off.
- **Return to rest** leaves mapping mode. **Park** also works from it.
- Stop pauses as usual, and **Resume** returns to mapping mode.

With **Build: Auto** (the default), the map grows only in mapping mode, including while paused in it, so the arms are never swept into the map. **On** maps all the time; **Off** only tracks.

**Not yet verified on the robot: whether the home pose actually keeps the arms out of the head camera's view.** Check the RGB feed the first time. As a second guard, the streamer ignores every point closer than 0.45 m to the camera (`--min-range` in `zed_stream.py`).

### How the map is built

- The ZED SDK tracks the camera with stereo images and its IMU (gravity-aligned world frame, z up, x forward, origin where the camera started).
- About 5 times a second, `zed_stream.py` takes a 640×360 point cloud in that world frame and drops points nearer than 0.45 m or farther than 4 m. It bins the rest into 2.5 cm voxels.
- A voxel is sent once it has been seen in two frames, which removes most flying pixels. Its averaged color is sent again at 8 and 32 sightings.
- The Mac keeps the voxel log and the page draws it with WebGL.
- **Limits of this first version:**
  - There is no loop closure, so a long drive accumulates drift.
  - Voxels are never removed, so people walking by leave trails.
  - The floor height in the 3D view is estimated from the lowest voxels, so the robot model sits on the floor once some floor has been mapped.
- **Save .ply** writes the map on the robot to `~/zed-setup/maps/map-<time>.ply`; open it in CloudCompare, MeshLab, or Open3D. Pass `--save-dir` to save elsewhere.
- **Video and control share one SSH connection.** Each stream keeps at most two frames in flight, waiting for the Mac's acknowledgment, so video cannot queue up ahead of control messages on a slow link. If the link chip still turns amber or red while the camera runs, stop the camera.

## Try the controller offline

Double-click **Try Controller Simulator.command**. A Terminal window starts the local controller and opens the page in your default browser. Keep the Terminal window open; closing it disconnects the controller. Click **Connect**, then **Enable**.

To try the dashboard, click **Start camera**, then **Mapping mode**. The simulated camera looks at a furnished room. The map starts building, and driving moves the camera.

To rehearse a chassis that does not respond, run `python3 mac_controller.py --simulation --sim-stall` from this folder.

## At the lab

1. Boot the robot, join the lab network, and bring up `can0`, `can1`, and `can5` (see Robot-side setup).
2. Double-click **Send Controller Update.command** the first time after a code change. It installs the files and opens the controller. Afterwards use **Open ARX Controller.command**.
3. If the robot's address changed, run from Terminal: `ARX_ROBOT=arx@<address> bash "Open ARX Controller.command"`.

## Controls

Control keys act as soon as you press them while the page has focus. **Esc** or **STOP** pauses at any time, and so does clicking another window or the page going silent for 0.5 s. A key that is already down when the robot becomes Ready (or after a stop or warning) is ignored until you let go of every key and press it again.

**Space (optional).** The **Space** setting in the input panel turns the old deadman back on: with **Hold to enable**, every control needs Space held, and releasing Space brakes the chassis and holds any unfinished jog. It is off by default and resets to off each time the app starts.

| Subsystem | Keys |
| --- | --- |
| Chassis | Hold **W/S** forward/back, **A/D** strafe, **←/→** turn. Combine keys for diagonals. |
| Arms | Tap **I/K** up/down, **J/L** left/right, **U/O** forward/back in 5 mm steps, up to 25 mm from the resting pose. **G/H** open/close grippers. |
| Torso & lift | Tap **Q/E** tilt/return toward upright and **R/F** raise/lower the lift. |
| Head | Tap **1/2** to jog head pitch. |

| Setting | Keys |
| --- | --- |
| Arm selection | **Z** left, **X** both, **C** right. Jogs and grippers apply to the selected arms. |
| Chassis speed | **−** / **=** cycles straight speeds of 0.72, 0.96, and 1.20 m/s along either axis. Turn rates are 1.125, 1.50, and 1.875 rad/s. |
| Brake behavior | In the Chassis panel. See below. |

### Lift ceiling (camera clearance)

A ZED Mini is mounted above the lift, so the lift must stop about 6 in (150 mm) short of the top. `LIFT_CEILING` in `jog.py` is a hard cap: `Hardware.body_tick` clamps every height sent to the SDK, and the controller and jog console both refuse jogs past it. The SDK's own travel is 0 to 20 units (`LIFT_SDK_MAX`).

The ceiling is set to **10.0 SDK units** (half travel) for now, because nobody has measured mm per SDK unit yet. To set the real value, measure the lift's full stroke in mm and compute `mm_per_unit = stroke / 20`. Then set `LIFT_CEILING = 20 - 150 / mm_per_unit`, or lower if the lift hides the camera's view before that height. The vendor VR script `robot-original/myx7s/vr_base.py` does not use this cap: it allows heights up to 22.

## What the buttons do

| Button | Available | What happens |
| --- | --- | --- |
| **Enable** | Standby | Opens both arm SDKs, then the body SDK. Homes both arms in parallel while the SDK calibrates the lift and torso with the chassis braked. Checks that the lift and torso read near zero (within 0.30 SDK units and 0.08 rad), records those readings as their floor, then moves to the resting pose: each arm 12 mm outward and 6 mm up from home, lift and torso held at their calibrated floors. On the robot the floor reads about 0.20 for the lift and 0.06 rad for the torso: both sit on their stops after calibration. The page shows a checklist and countdown (30 s to finish homing, 6 s to settle at rest). |
| **STOP / Esc** | Connected | Ready, Paused, Parking: brakes, holds arms at their current joints and the body where it is, and goes to **Paused**. Startup: becomes a fault, which **Recover** can restart. |
| **Resume** | Paused | Returns to Ready without moving. |
| **Park** | Ready, Paused | Arms go home, and the torso and lift return to their calibrated floor (upright, lift at the bottom). Shows **Parked** once feedback confirms it. **Return to rest** goes back to the resting pose. |
| **Mapping mode** | Ready, Paused | Brakes, folds both arms home and returns the torso upright, leaving the lift where it is. Shows **Mapping mode** once the arms are home. In it, only driving, lift, and head pitch work. **Return to rest** goes back to the resting pose. See Dashboard. |
| **Recover** | Fault | Startup finished: re-checks all feedback and holds the current pose, then goes to **Paused**. Startup unfinished: restarts homing with the controllers already open. Not offered if an SDK process has died. |
| **Disconnect / Quit app** | Connected | Ends the robot session. **The SDK can release motor torque at disconnect.** When the robot is live and not parked, the page offers **Park first**. |
| **Take over robot** | After a "still running" message | Stops a previous controller session that is still holding the robot (for example after a network drop). This releases that session's torque, so support the robot first. |

A fault keeps the robot braked and holding wherever it can. Reconnecting is no longer the only way out.

## Chassis behavior

- **Smooth stop (default).** While a drive key is held, the chassis is in SDK drive mode 1 and the keys set the target velocity. When you let go, the base stays in drive mode for 0.5 s while it slows to zero, then switches to brake mode 2. Stop, focus loss, or late input switch to brake mode 2 immediately. This matches how the vendor's VR and keyboard examples use modes 1 and 2. With Space required, this setting is called **On Space release**: drive mode stays on the whole time Space is held, and releasing Space brakes.
- **Brake on key release** restores the previous behavior: mode 2 whenever no drive key is held. Use it if mode 1 at zero velocity behaves badly on the hardware.
- **Speed ramp.** Speeding up is limited to 0.6 m/s² (1.2 rad/s² turning). Slowing down is limited to 1.5 m/s² (3.0 rad/s²). Braking with mode 2 skips the ramp.
- **Speeds clear the observed deadband.** 0.048 m/s (16 chassis counts) did not move the base and 0.15 m/s (49 counts) did. The old 0.12 rad/s turn was only 19 counts and diagonals were 35 counts per axis. Every axis now sends at least 49 counts. Diagonals are capped at 0.30 m/s total.
- **Wheel checks.** Driving needs fresh wheel feedback. If the wheels report no motion during a 1 s drive, the page shows **No wheel response**. Until the wheel reports have shown motion once in the session this is only a warning, because the feedback has not yet been seen during real motion. After that, it brakes and asks you to release the keys (or Space). Wheels reporting motion while braked for more than 1 s raise a warning.

## Timeouts

| Where | Limit | Effect |
| --- | --- | --- |
| Page → Mac app | 0.20 s | Page input older than this counts as released. |
| Page → Mac app | 0.50 s | A silent page counts as unfocused. In Ready this pauses the robot. |
| Robot → Mac app | 0.6 s (6 s during startup) | The Mac sends Stop and shows **Connection interrupted**. |
| Mac → robot | 0.30 s | Oldest status a reply may answer. Older replies are ignored. |
| Mac → robot | 0.25 s | Chassis brakes. The session stays Ready. |
| Mac → robot | 1.0 s | Robot pauses (or a startup fault). |
| Mac → robot | 60 s with no bytes at all | The bridge exits, releasing torque and the lock, so a dead link cannot hold the robot forever. |

## CAN monitor

The **CAN monitor** panel in the page runs the read-only `can_monitor.py` on the robot over the same SSH session. It shows wheel speed reports, the host chassis command (mode and velocity), and raw lift, waist, head yaw, and head pitch motor feedback. Body angles are raw motor angles before the SDK unwraps them and do not measure lift travel.

## Robot-side setup

On the robot, `~/Documents/Teleop Test Enviroment/.deps` contains the test project's pinned NumPy, SciPy, and Pygame packages. `run-robot.sh` and `run-controller.sh` use that persistent directory. `bash prepare-runtime.sh` is also available for a temporary test copy in RAM.

For a powered test, first confirm the drive is suitable for operation, the work area is clear, a spotter has the physical stop, and the original controllers are stopped. The ARX SDK can move or calibrate actuators while starting.

The whole-robot controller needs all three serial-based CAN adapters up: `can5` for base, lift, and head, `can0` for the left arm, and `can1` for the right arm. The installed mapping is backed up in `robot-original/can/arx_can.rules`. For a missing interface, use the matching device and interface:

```bash
sudo modprobe slcan
sudo slcand -o -f -s8 /dev/arxcan5 can5
sudo ip link set dev can5 up
ip -brief link show dev can5
```

If the interface exists and is down, use only `sudo ip link set dev can5 up` (or the matching arm interface).

### Deploying and rolling back

**Send Controller Update.command** packs `jog.py`, `robot_session.py`, `control_bridge.py`, `can_monitor.py`, `zed_stream.py`, `run-controller.sh`, `run-robot.sh`, and `prepare-runtime.sh`, checks their SHA-256 sums on the robot, and compiles the Python files with the robot's Python. Only then does it replace anything. Each replaced file is first copied to `~/Documents/Teleop Test Enviroment/.controller-backup/<date-time>/`. The update refuses to run while a controller or jog console holds the lock.

To roll back on the robot, copy the files from the newest backup folder over the project files. The previous Mac controller is in the baseline git commit. The old and new versions do not talk to each other: the page says so if the robot still has the old controller.

## Jog console

`python3 jog.py base` (or `left`, `right`, `lift`, `head`) runs a simulated single-subsystem console with Python's standard library. On the robot, `bash run-robot.sh base --live` asks for `LIVE base` before opening the SDK. Base pulses are 0.25 s followed by braking. Its turn speed is now 0.30 rad/s to clear the deadband.

## Tests

From this folder: `python3 -m unittest discover -s tests -t .`. The tests cover startup, chassis modes and ramps, wheel checks, jogs and limits, park, fault recovery, leases and timeouts, the Mac input rules, the page server's token checks, mapping mode, the camera message format, the map and image endpoints, the simulated camera, and an end-to-end simulator run through a real bridge process. They do not exercise the ARX SDK, CAN hardware, or the ZED SDK. The voxel map test needs NumPy and is skipped on a Mac without it.

## SSD finding

The Kingston NVMe drive logged unrecovered media reads. The original SciPy LAPACK library returned an I/O error, which caused the arm controller to crash during import. The November 12 checkout recorded 5,311 media or data-integrity errors. Preserve other important robot data and replace or have the drive evaluated before depending on it. [NVMe SMART log field definitions](https://nvmexpress.org/wp-content/uploads/NVM-Express-Base-Specification-Revision-2.3-2025.08.01-Ratified.pdf)
