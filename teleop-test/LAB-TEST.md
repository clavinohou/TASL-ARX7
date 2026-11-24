# Lab test plan: controller update (November 2026)

This is the first hardware run of the new controller: the browser page, the latched chassis mode, the new turn and diagonal speeds, the speed ramp, wheel checks, Park, Recover, takeover, and single-arm jogs. Everything below passed in the simulator only. Go step by step and stop at the first surprise.

**Before every powered step:** the robot is in view, the area is clear (at least 2 m around the base for chassis tests), a spotter has the physical stop, and someone can support the arms, lift, and torso.

Record what you see in the right-hand column, or in a copy of this file.

## 0. Before leaving (Mac)

| Step | Expected | Result |
| --- | --- | --- |
| In Terminal, from `teleop-test`: `python3 -m unittest discover -s tests -t .` | `OK` (97 tests, 1 skipped) | |
| Double-click **Try Controller Simulator.command**, then Connect → Enable → W → Park | Page opens and works as in the README | |

## 1. Network and CAN

| Step | Expected | Result |
| --- | --- | --- |
| Boot the robot, join the lab network. In Terminal: `ssh arx@10.90.134.126 true` | Signs in. If the address changed, use `ARX_ROBOT=arx@<address>` for every launcher (README, At the lab). | |
| On the robot: `ip -brief link show` | `can0`, `can1`, `can5` all `UP`. Otherwise use the slcand steps in the README. | |
| Make sure the original VR controllers and any jog console are stopped | No `vr_base.py`, `vr_dual_arm.py`, `jog.py` processes (`pgrep -af "vr_|jog.py"`) | |

## 2. Deploy

| Step | Expected | Result |
| --- | --- | --- |
| Double-click **Send Controller Update.command** and sign in | `Updated …`, `Previous versions saved in …/.controller-backup/<time>`, `Controller update installed and verified`, and `can0/can1/can5 is up` | |
| The page opens. Check the header | Red **LIVE · arx@…** chip. Link chip shows `No link`. | |

If the update stops with a compile error, the robot's Python rejected a file. Nothing was replaced. Note the message and stop.

## 3. Connect and startup

| Step | Expected | Result |
| --- | --- | --- |
| **Connect** | Phase **Standby**. Link chip turns green with RTT and about 20 Hz. Note the RTT. | RTT: |
| **Enable** → confirm | Phase **Starting up** with a checklist and a countdown. Both arms home together, then the lift and torso calibrate. | |
| Wait | Checklist completes, then **Moving to rest**, then **Ready** within about 30 s. Arms end about 12 mm outward and 6 mm up from home, and the lift rises slightly. | Time to Ready: |
| If it faults | Read the reason in the Events panel. Press **Recover** once the cause is clear. Startup restarts. | |

## 4. Stop, resume, focus

| Step | Expected | Result |
| --- | --- | --- |
| Press **Esc** | **Paused**, robot holds still | |
| **Resume** | **Ready**, no motion | |
| Click another app (for example Finder), then come back | Paused with "lost focus" in Events. Resume. | |

## 5. Chassis (speed 0.15 m/s, Space: Not required, Brake: Smooth stop)

Do every chassis step with short presses first (under 1 s). Watch the wheel bars and the Events panel. Keys now act without Space, so keep your hands off the keyboard except for the step you are doing.

| Step | Expected | Result |
| --- | --- | --- |
| Robot Ready, no key pressed, 3 s | Chassis tag **Braked**. The base does not move. | |
| **W** for about 1 s, then let go | Moves forward, slows to a stop, and the tag returns to **Braked** within about 0.5 s. **Note any creep** in that half second (the base is in drive mode 1 at zero velocity). If it creeps, switch Brake to **On key release** and note it. | |
| **S**, **A**, **D** the same way | Back, strafe left, strafe right | |
| **←** about 1 s | **Turns left (counterclockwise from above)** at 0.30 rad/s. The old 0.12 rad/s probably did not move at all. | |
| **→** | Turns right | |
| **W + A** | Moves diagonally forward-left. Note if only one direction moves. | |
| While moving with W, press **Esc** | Immediate brake, **Paused**. Resume, release W, press it again to drive. | |
| While moving with W, click another window | Immediate brake, Paused with "lost focus". Come back and Resume. | |
| Hold W, press Esc, Resume while still holding W | Nothing moves until W is released and pressed again | |
| Wheel bars W1–W4 during motion | Nonzero values. Note the size at 0.15 m/s. If they stay at 0 while the base moves, the page warns **No wheel response**. Note it: the wheel feedback is not working. | W values: |
| Optional: speed 0.20, then 0.25 (**=** key) | Faster, still smooth | |
| Optional: Brake **On key release**, W | Brakes as soon as W is released | |
| Optional: Space **Hold to enable**. Hold **Space** only, for 3 s | Chassis tag **Drive mode**. **The base must not move or creep.** Then Space + W works, and releasing Space brakes at once. Switch back to **Not required** afterwards. | |

## 6. Arms, body, grippers

| Step | Expected | Result |
| --- | --- | --- |
| Arm selection **Left** (Z). Tap **I** | Only the left arm rises 5 mm. The left plot's Z bar moves. | |
| **Both** (X). Tap **J**, then **L** | Both arms move left, then back | |
| Tap **U** repeatedly | Stops at 25 mm with "jog limit reached" | |
| Tap **R** once, then **F** | Lift up 0.10, then back down | |
| Tap **Q** once, then **E** | Torso tilts 0.02 rad, then returns | |
| Tap **1**, then **2** | Head pitch moves and returns | |
| **G**, then **H** (Both) | Both grippers open, then close | |

## 7. Park and disconnect

| Step | Expected | Result |
| --- | --- | --- |
| **Park** → confirm | **Parking**, then **Parked**: arms home, torso upright, lift at the bottom | |
| **Return to rest** | Back to **Ready** at the resting pose | |
| **Park** again | **Parked** | |
| **CAN monitor** → Start | Wheel reports and the four body motors update. Stop it. | |
| Support the arms, lift, and torso. **Disconnect** → confirm | Note whether torque releases and anything drops | |

## 7b. Camera, dashboard, and mapping mode

Needs the ZED stack installed and `zed_check.py` passing (camera/README.md). Do this after step 6 with the robot at rest.

| Step | Expected | Result |
| --- | --- | --- |
| **Start camera** (Live map panel) | Within about 10 s the RGB feed (top right) and depth view (bottom right) go live, and the chip reads **Tracking**. The link chip stays green. Note RTT with the camera running. | RTT: |
| Watch the link chip for 30 s | Stays green or amber. If it turns red or shows "No status", stop the camera and note it: video is starving the control link. | |
| **Mapping mode** → confirm | **Folding arms**, then **Mapping mode**. Arms at home, torso upright, lift unchanged. | |
| Check the RGB feed | **Are any parts of the arms or grippers visible?** Note where. If they are, the home pose is not enough and the map will pick them up past 0.45 m. | Arms visible? |
| Map panel | Chip **Mapping**, voxel count rising | |
| Slow turn in place (←), then a short W | The 3D view shows the room filling in around the robot model, and the model turns and moves with the base | |
| Tap **I** (arm key) | Ignored with "off in mapping mode" | |
| **Save .ply** | Event shows the saved path under `~/zed-setup/maps/` | Path: |
| **Return to rest** → confirm | Back to **Ready** at the resting pose. With Build: Auto the voxel count stops rising. | |
| **Stop camera** | Feeds go blank, the map stays on screen | |

## 8. Optional: takeover

Only if there is time. With the robot **Parked** and supported:

| Step | Expected | Result |
| --- | --- | --- |
| Connect → Enable → Park. Turn off the Mac's Wi-Fi for about 5 s, then turn it back on | Page shows **Connection interrupted**. The robot pauses after 1 s and keeps holding. | |
| If the SSH link dropped (page shows **Disconnected**), click **Connect**. If the Terminal asks for the robot password, enter it there. | "A previous controller session is still running…" and a **Take over robot** button. If the old link survived instead, the page recovers on its own and shows **Paused**: Resume and skip the next row. Otherwise the old session exits by itself within 60 s. | |
| Support the robot. **Take over robot** | Old session stops (torque may release). New session reaches **Standby**. | |

## If something goes wrong

- **Physical stop first.** The software stop depends on the SDK and the link.
- The Events panel keeps the robot's messages, and the Terminal window shows the local app's output. Copy both into your notes.
- **Rollback:** on the robot, copy the newest `.controller-backup/<time>/` files back into `~/Documents/Teleop Test Enviroment/`. On the Mac, the previous controller is in the baseline git commit (`git log`). Roll back both sides together.
