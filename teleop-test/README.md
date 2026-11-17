# ARX X7s teleop test

`robot-original/myx7s` is the lab control stack copied from the robot. It includes the Linux Python 3.10 arm and lift SDK, robot models, VR receiver, and original launcher. The CAN mapping and older examples are under `robot-original`. The Mac has a separate backup of the selected control files and CAN rule.

## Try the console

Run `python3 jog.py base` or replace `base` with `left`, `right`, `lift`, or `head`. Simulation is the default and uses Python's standard library. Enter one command at a time.

| Mode | Commands |
| --- | --- |
| Base | `w/s` forward and back, `a/d` strafe, `e/r` turn. One 0.25-second pulse followed by braking. |
| Left or right arm | `x+`, `x-`, `y+`, `y-`, `z+`, `z-`. Each jog is 5 mm in the arm frame with a 25 mm session radius. |
| Lift or head | `+` and `-` within a small range around the starting height or pitch. |
| Any mode | `stop` to brake or hold, `q` to quit. |

The jog console covers chassis movement, both arm positions, lift, and head pitch. The original SDK also contains waist, head yaw, and gripper controls.

## Mac keyboard controller

Double-click `Try Controller Simulator.command` on the Mac to practice offline. Click **Connect**, then **Enable**. The simulator shows each arm homing, the lift and torso zeroing, and the move to the resting pose. No robot is needed.

At the lab, boot the robot, connect to the lab network, and double-click `Send Controller Update.command`. Enter the SSH password if prompted. The sender verifies the installed files and opens the updated controller. Click **Connect** to reach standby. Run the same sender again if the transfer is interrupted. On later visits use `Open ARX Controller.command`. The controller connects over SSH and works with GNOME unavailable.

When the robot is in view, the area is clear, and the physical stop is within reach, press **Enable**. Both arms start their native homing sequences in parallel. The body SDK calibrates the lift and waist pivot during the same startup sequence, with the chassis braked. The controller waits for fresh feedback before requesting the resting pose and again before unlocking manual controls. The initial rest uses the SDK's upright torso zero, a lift target of 0.10 in SDK units, and each arm's measured home tool pose offset 12 mm outward and 6 mm upward. Verify clearance, directions, and the resting pose on the first supervised lab run.

| Subsystem | Keyboard controls while holding Space |
| --- | --- |
| Chassis | Hold **W/S** for forward/back, **A/D** to strafe, or **←/→** to turn. Translation is limited to 0.15 m/s. |
| Both arms | Tap **I/K** for up/down, **J/L** for left/right, **U/O** for forward/back. Both tools move by the same 5 mm increment while retaining their own orientation. Travel is bounded to 25 mm from each resting pose. |
| Torso & lift | Tap **Q/E** to tilt/return toward upright and **R/F** to raise/lower the rail. |
| Grippers & head | Tap **G/H** to open/close both grippers or **1/2** to jog head pitch. |

Release the drive key to brake or release **Space** to brake and request a hold for any unfinished jog. **Esc** or **Stop** locks manual input and requests a position hold. **Enable** resumes from a pause. An interrupted or failed startup requires reconnecting. The robot expires control input after 250 ms. The SDK may release torque at disconnect, so support the arms, lift, and torso before disconnecting. The physical stop is the way to halt movement if the SDK or an actuator fails.

The earlier 0.15 m/s forward diagnostic moved the physical robot. The redesigned controller, its resting pose, and the new WASD and arrow mappings still need a supervised hardware test.

### Encoder monitor

Run `bash 'Open ARX Controller.command' --encoders` from this directory. The read-only `can5` monitor can run beside the controller. It shows four wheel speed reports, the last nonzero chassis command, and lift, waist pivot, head yaw, and head pitch motor feedback. The wheel values are angular velocity. The body readings are raw motor angles before the SDK unwraps them and do not measure lift travel or confirm homing. A blank value means no fresh report was received. The monitor records set-zero and disable commands during the session. Keep keyboard focus on the Controller window when testing. The active keycaps and wheel readout also appear in the controller.

## Robot-side setup

On the robot, `~/Documents/Teleop Test Enviroment/.deps` contains the test project's pinned NumPy, SciPy, and Pygame packages. `run-robot.sh` uses that persistent directory. `bash prepare-runtime.sh` is also available for a temporary test copy in RAM.

For a powered test, first confirm the drive is suitable for operation, the work area is clear, a spotter has the physical stop, and the original controllers are stopped. The ARX SDK can move or calibrate actuators while starting. Verify the physical stop before opening the console.

The whole-robot controller needs all three serial-based CAN adapters up. `can5` is shared by base, lift, and head. `can0` is left arm. `can1` is right arm. The installed mapping was backed up in `robot-original/can/arx_can.rules`. For a missing interface, use the matching device and interface in the one-shot setup below.

```bash
sudo modprobe slcan
sudo slcand -o -f -s8 /dev/arxcan5 can5
sudo ip link set dev can5 up
ip -brief link show dev can5
```

The CAN interfaces may need to be brought up again after a reboot. If the sender reports a missing interface, use the commands above in a terminal on the robot with the matching interface number. If the interface exists and is down, use only `sudo ip link set dev can5 up` or the matching arm interface. Once the controller shows **Ready**, use a brief **Space + W** press and verify that releasing either key stops the wheels. Check **A**, **S**, **D**, and the left and right arrows individually while watching the wheel readout.

From an interactive SSH or robot desktop terminal, change to `~/Documents/Teleop Test Enviroment` and run `bash run-robot.sh base --live`. The console asks for `LIVE base` before opening the SDK. Start with one jog and verify direction and the stop behavior on the physical robot before proceeding. The pulse timer and shutdown handler are software controls. Keep the physical stop within reach.

## SSD finding

The Kingston NVMe drive logged unrecovered media reads. The original SciPy LAPACK library returned an I/O error, which caused the arm controller to crash during import. The November 12 checkout recorded 5,311 media or data-integrity errors. Preserve other important robot data and replace or have the drive evaluated before depending on it. [NVMe SMART log field definitions](https://nvmexpress.org/wp-content/uploads/NVM-Express-Base-Specification-Revision-2.3-2025.08.01-Ratified.pdf)
