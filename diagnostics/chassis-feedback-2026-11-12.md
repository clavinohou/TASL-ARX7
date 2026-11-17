# Chassis and body feedback, November 12, 2026

The supervised Space + W retest was monitored passively on `can5`. The host emitted **23,553 chassis frames**, all sampled commands were mode 2 (brake), and no nonzero drive frame was observed. The chassis returned **651 wheel-speed frames** with all four wheel speeds at `0.000 rad/s`. This confirms communication with the chassis controller and that it reported stationary wheels. A physical mode-1 drive command still needs to be observed before assessing the wheel motors. [Recorded feedback snapshots](chassis-retest.9luWpc)

The same installed SDK and control bridge, run in a Linux network namespace containing only loopback and a virtual CAN device, emitted a nonzero forward frame in mode 1 and a zero-velocity brake frame in mode 2. The Mac window separately passed a simulated Space + W press and W release. A keyboard indicator was added to the Mac controller to check the real key and focus state during the next test.

| SDK assignment | Motor ID | Raw motor angle during the chassis test | Reported status |
| --- | --- | --- | --- |
| Lift | 1 | −0.267 rad | 1 |
| Waist pivot | 4 | −1.102 rad | 1 |
| Head yaw | 5 | +0.052 to +0.053 rad | 1 |
| Head pitch | 6 | −0.292 rad | 1 |

The four IDs answered on the expected body bus. The angles remained essentially stationary. These are raw motor angles and cannot confirm physical homing or establish a lift height. The installed SDK has separate lift and waist calibration logic. It sends body motor enable commands during startup and disable commands on shutdown or in protection mode. The chassis retest captured no set-zero or disable frames. An additional trace starting before body-controller activation is needed to determine whether calibration completes or the motors disable during the reported coasting. No homing targets, gains, CAN mappings, or motion limits were changed during diagnosis.

## Follow-up with keyboard input confirmed

The live monitor showed **6,012 nonzero mode-1 chassis frames** with a last forward command of approximately **0.048 m/s** after SDK quantization. Peak feedback remained **0.000 rad/s on all four wheels**. The host transmitted a valid-shaped drive request on `can5` and received wheel-speed feedback. The monitor cannot verify whether the chassis firmware accepted the command or whether a drive inhibit or low-speed threshold prevented motion. The earlier brake-only trace describes a separate attempt.

The monitor also saw set-zero requests for the lift and waist and four body motor disable requests over its lifetime. Those counters do not establish whether the disables occurred during startup, a protection event, or an earlier session shutdown. Current body feedback was present after the user's attempt.

The ARX manual on the robot documents a separate chassis unlock in the VR workflow. The installed ARX joystick example uses mode 1 for driving and mode 2 for braking. The lab's keyboard example defaults to 0.5 m/s. The updated Mac controller keeps its 0.05 m/s normal setting and adds a one-shot, 0.25-second forward diagnostic at 0.15 m/s. The installed SDK emitted the diagnostic drive frames and then brake frames on the isolated virtual CAN bus. Physical response to that diagnostic remains untested.

## Post-calibration hold

After the user tried Space + T, the monitor still showed 0.048 m/s as the last nonzero command and 0.000 rad/s as the peak speed for each wheel. There was no confirmation of the higher-speed pulse. The controller now counts accepted T pulses.

The previous controller saved the lift and waist position before the SDK established its zero reference. Switching between Chassis, Lift, and Head pitch also ended the SDK session. The installed SDK destructor sends motor disable frames. The updated controller waits for each SDK set-zero frame and fresh enabled feedback before capturing a hold target. It shares the SDK session across the three body tabs and keeps holding through Stop. The isolated SDK test with simulated encoders observed both set-zero frames, a diagnostic chassis pulse, and 294 subsequent body control frames through Stop and a tab change, with no disable frame during that interval. Physical holding has yet to be verified. The SDK's shutdown and protection paths can still release torque.

## WASD update for the next lab visit

Calvin observed physical movement with the 0.15 m/s Space + T pulse. The staged update uses that same speed for all four WASD directions. WASD drives while the operator holds Space and a direction. Releasing either key requests braking, and loss of fresh input expires the command after 250 ms. The robot shut down before this update could be deployed. [Installation and test steps](../teleop-test/README.md#mac-keyboard-controller)

The Mac tests exercised all four directions, key release, the T pulse timer, and input timeout through the controller's message loop. A local copy of the robot's last verified file versions passed installation, checksum verification, and retry after a partial install. The sender also stopped when an existing controller held the lock or either installed file had an unexpected edit. The next lab visit is the first test of this WASD update on hardware.
