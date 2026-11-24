from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import select
import signal
import socket
import sys
import time

from can_monitor import FILTER, FRAME

INTERFACES = {"base": "can5", "lift": "can5", "head": "can5",
              "left": "can0", "right": "can1"}
BODY_MODES = ("base", "lift", "head")
LINEAR_SPEED = 0.15  # m/s, verified by the supervised forward pulse
DRIVE = {
    "w": (LINEAR_SPEED, 0.0, 0.0),
    "a": (0.0, LINEAR_SPEED, 0.0),
    "s": (-LINEAR_SPEED, 0.0, 0.0),
    "d": (0.0, -LINEAR_SPEED, 0.0),
    # 0.12 rad/s quantized to 19 chassis counts, close to the 16 counts that
    # did not move the base. 0.30 rad/s is 49 counts, like the 0.15 m/s pulse.
    "e": (0.0, 0.0, 0.30),
    "r": (0.0, 0.0, -0.30),
}
ZERO = (0.0, 0.0, 0.0)
PULSE_SECONDS = 0.25
BRAKE_SECONDS = 0.15
TICK_SECONDS = 0.002
ARM_STEP_METERS = 0.005
ARM_RADIUS_METERS = 0.025
# Lift height is in SDK units: the SDK clamps it to 0..20 (max_height_ in
# lift_head_control_loop.h). The ZED Mini mounted above the lift needs about
# 6 in (150 mm) of clearance at the top, so every height sent to the SDK is
# capped at LIFT_CEILING. 10.0 is a conservative placeholder until the lift's
# mm per SDK unit is measured; then use 20 - 150 / mm_per_unit.
LIFT_SDK_MAX = 20.0
LIFT_CEILING = 10.0
BODY_JOG = {"lift": (0, 0.1, 0.5, 0.0, LIFT_CEILING),
            "head": (3, 0.02, 0.08, 0.0, 0.8)}
ZERO_AXES = {1: (0, LIFT_SDK_MAX), 4: (1, 3.14)}
BODY_FEEDBACK_TIMEOUT = 0.25
ZERO_SETTLE_SECONDS = 0.06
HELP = {
    "base": "w/s forward/back, a/d strafe, e/r turn",
    "lift": "+/- raise/lower",
    "head": "+/- change pitch",
    "left": "x+/x-  y+/y-  z+/z- jog the left tool in its arm frame",
    "right": "x+/x-  y+/y-  z+/z- jog the right tool in its arm frame",
}


def checked_pose(values):
    pose = tuple(float(value) for value in values)
    if (len(pose) != 7 or not all(math.isfinite(value) for value in pose)
            or not 0.95 <= math.hypot(*pose[3:]) <= 1.05):
        raise ValueError("Arm feedback did not contain a valid position and quaternion")
    return pose


class BodyCalibration:
    def __init__(self, interface):
        self.bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        mask = socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG | socket.CAN_SFF_MASK
        self.bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                            b"".join(FILTER.pack(ident, mask) for ident in (0, *ZERO_AXES)))
        self.bus.bind((interface,))
        self.bus.setblocking(False)
        self.zeroed = {}
        self.holding = set()
        self.feedback = {}
        self.disabled = False

    def read(self, now):
        for _ in range(128):
            try:
                frame, _, flags, _ = self.bus.recvmsg(FRAME.size)
            except BlockingIOError:
                break
            ident, length, data = FRAME.unpack(frame)
            if length != 8:
                continue
            if flags & socket.MSG_DONTROUTE:
                if ident in ZERO_AXES and data[:7] == b"\xff" * 7:
                    if data[7] == 0xfe:
                        self.zeroed[ident] = now
                        self.holding.discard(ident)
                        self.feedback.pop(ident, None)
                    elif data[7] == 0xfd:
                        self.disabled = True
            elif ident == 0 and data[0] & 0x0f in self.zeroed:
                motor = data[0] & 0x0f
                if data[0] >> 4 == 1 and now - self.zeroed[motor] >= ZERO_SETTLE_SECONDS:
                    self.feedback[motor] = now
                else:
                    self.feedback.pop(motor, None)

    def fresh(self, motor, now):
        return now - self.feedback.get(motor, -math.inf) < BODY_FEEDBACK_TIMEOUT

    def ready(self, now):
        return not self.disabled and all(motor in self.holding and self.fresh(motor, now) for motor in ZERO_AXES)


class Hardware:
    def __init__(self, mode):
        interface = INTERFACES[mode]
        link = Path("/sys/class/net") / interface
        if not (link / "flags").is_file() or not int((link / "flags").read_text(), 16) & 1:
            raise RuntimeError(f"Bring up {interface} before opening the controller.")
        control_dir = Path(__file__).resolve().parent / "robot-original" / "myx7s"
        sys.path.insert(0, str(control_dir))
        self.body = None
        self.arm = None
        self.calibration = None
        if mode in ("left", "right"):
            from bimanual import SingleArm

            self.arm = SingleArm({"can_port": interface, "type": int(mode == "right")})
        else:
            import lift_controller
            import arx_lift_python

            self.calibration = BodyCalibration(interface)
            self.body = arx_lift_python.LiftHeadControlLoop(
                interface, arx_lift_python.RobotType.X7S
            )

    def initial_body_position(self):
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            self.body.read()
            time.sleep(TICK_SECONDS)
        _, _, yaw, pitch = self.body_position()
        return (0.0, 0.0, yaw, pitch)

    def body_position(self):
        position = (self.body.get_height(), self.body.get_waist_pos(),
                    self.body.get_head_yaw(), -self.body.get_head_pitch())
        if not all(math.isfinite(value) for value in position):
            raise ValueError(f"Invalid body feedback: {position}")
        return position

    def post_zero_positions(self, now):
        calibration = self.calibration
        calibration.read(now)
        waiting = calibration.zeroed.keys() - calibration.holding
        if not waiting or calibration.disabled:
            return {}
        self.body.read()
        position = self.body_position()
        captured = {}
        for motor in waiting:
            axis, maximum = ZERO_AXES[motor]
            value = position[axis]
            if calibration.fresh(motor, now) and -0.02 <= value <= maximum:
                captured[axis] = max(0.0, value)
                calibration.holding.add(motor)
        return captured

    def body_tick(self, position, velocity, mode, head_target=None):
        """Mode 1 is the SDK's drive (unlocked) state and mode 2 its brake."""
        height, waist, _, _ = position
        # Hard limit for every caller (jog console and whole-robot controller).
        self.body.set_height(min(height, LIFT_CEILING))
        self.body.set_waist_pos(waist)
        if head_target is not None:
            self.body.set_head_pitch(head_target)
        self.body.set_chassis_cmd(*velocity, mode)
        self.body.loop()

    def arm_pose(self):
        return checked_pose(self.arm.get_ee_pose())

    def move_arm(self, pose):
        self.arm.set_ee_pose(pos=pose[:3], quat=pose[3:])


class JogSession:
    def __init__(self, mode, hardware=None):
        self.mode = mode
        self.hardware = hardware
        self.velocity = ZERO
        self.pulse_end = 0.0
        self.brake_end = 0.0
        self.arm_moved = False
        self.body_moved = False
        self.head_target = None
        self.is_arm = mode in ("left", "right")
        if self.is_arm:
            self.anchor = hardware.arm_pose() if hardware else (0.3, 0.0, 0.25, 1.0, 0.0, 0.0, 0.0)
        else:
            self.anchor = hardware.initial_body_position() if hardware else (10.0, 0.0, 0.0, 0.3)
        self.target = self.anchor

    def drive_command(self, now):
        return (self.velocity, True) if now < self.pulse_end and self.body_ready(now) else (ZERO, False)

    def body_ready(self, now):
        return not self.hardware or self.is_arm or self.hardware.calibration.ready(now)

    def body_status(self, now):
        if self.is_arm or not self.hardware:
            return None
        calibration = self.hardware.calibration
        if calibration.disabled:
            return "Body motors disabled by SDK"
        if self.body_ready(now):
            return "Lift and waist hold commanded"
        if len(calibration.holding) == len(ZERO_AXES):
            return "Body motor feedback stale"
        return "Waiting for lift and waist zero feedback"

    def select_body_mode(self, mode):
        self.stop()
        self.mode = mode

    def tick(self, now):
        if self.hardware and not self.is_arm:
            captures = self.hardware.post_zero_positions(now)
            if captures:
                self.anchor = tuple(captures.get(index, value) for index, value in enumerate(self.anchor))
                self.target = tuple(captures.get(index, value) for index, value in enumerate(self.target))
                self.pulse_end = 0.0
            velocity, driving = self.drive_command(now)
            self.hardware.body_tick(self.target, velocity, 1 if driving else 2, self.head_target)

    def stop(self):
        self.pulse_end = 0.0
        if self.hardware and self.is_arm and self.arm_moved:
            self.target = self.hardware.arm_pose()
            self.hardware.move_arm(self.target)
            self.arm_moved = False
        if self.hardware and self.body_moved:
            if self.body_ready(time.monotonic()):
                actual = self.hardware.body_position()
                target = list(self.target)
                axis, _, _, minimum, maximum = BODY_JOG[self.mode]
                if minimum <= actual[axis] <= maximum:
                    target[axis] = actual[axis]
                    self.target = tuple(target)
                    if self.mode == "head":
                        self.head_target = target[axis]
            self.body_moved = False

    def close(self):
        try:
            self.stop()
        finally:
            self.brake()

    def brake(self):
        if self.hardware and not self.is_arm:
            deadline = time.monotonic() + BRAKE_SECONDS
            while time.monotonic() < deadline:
                self.tick(time.monotonic())
                time.sleep(TICK_SECONDS)

    def command(self, command, now):
        if command == "stop":
            self.stop()
            return "Stopped drive or holding the current actuator target."
        if not self.body_ready(now):
            return self.body_status(now)
        if self.mode == "base" and command in DRIVE:
            if now < self.brake_end:
                return "Wait for the previous pulse and brake interval."
            self.velocity = DRIVE[command]
            self.pulse_end = now + PULSE_SECONDS
            self.brake_end = self.pulse_end + BRAKE_SECONDS
            return f"Drive pulse {self.velocity} for {PULSE_SECONDS:.2f} s, then brake."
        if self.mode in BODY_JOG and command in ("+", "-"):
            axis, step, limit, minimum, maximum = BODY_JOG[self.mode]
            if self.mode == "head" and self.hardware and self.head_target is None:
                initial = self.hardware.body_position()[axis]
                if not minimum <= initial <= maximum:
                    return "Measured head pitch is outside the jog range."
                self.anchor = (*self.anchor[:axis], initial)
                self.target = (*self.target[:axis], initial)
            goal = self.target[axis] + (step if command == "+" else -step)
            if not minimum <= goal <= maximum or abs(goal - self.anchor[axis]) > limit + 1e-9:
                return "Target is outside this session's jog range."
            target = list(self.target)
            target[axis] = goal
            self.target = tuple(target)
            if self.mode == "head" and self.hardware:
                self.head_target = goal
            self.body_moved = True
            return f"{self.mode.capitalize()} target {goal:.3f}."
        if self.is_arm and command in ("x+", "x-", "y+", "y-", "z+", "z-"):
            current = self.hardware.arm_pose() if self.hardware else self.target
            target = list(current)
            axis = "xyz".index(command[0])
            target[axis] += ARM_STEP_METERS if command[1] == "+" else -ARM_STEP_METERS
            if math.dist(target[:3], self.anchor[:3]) > ARM_RADIUS_METERS + 1e-9:
                return "Target is outside the 25 mm jog radius."
            self.target = tuple(target)
            if self.hardware:
                self.hardware.move_arm(self.target)
                self.arm_moved = True
            return "Tool target " + ", ".join(f"{value:.3f}" for value in self.target[:3]) + " m."
        return HELP[self.mode]


def console(session):
    print(HELP[session.mode])
    print("Enter one command at a time. Type stop to stop a jog and q to quit.")
    if session.is_arm:
        print("Starting tool position " + ", ".join(f"{value:.3f}" for value in session.anchor[:3]) + " m.")
    pending = b""
    while True:
        session.tick(time.monotonic())
        interval = TICK_SECONDS if session.hardware and not session.is_arm else 0.1
        readable, _, _ = select.select([sys.stdin], [], [], interval)
        if not readable:
            continue
        data = os.read(sys.stdin.fileno(), 4096)
        if not data:
            return
        lines = (pending + data).split(b"\n")
        pending = lines.pop()
        if session.hardware and len(lines) > 1:
            session.stop()
            print("Enter a single command for each jog.", flush=True)
            continue
        for line in lines:
            command = line.decode(errors="replace").strip().lower()
            if command == "q":
                return
            if command:
                print(session.command(command, time.monotonic()), flush=True)


def interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description="ARX X7s actuator jog test")
    parser.add_argument("mode", choices=INTERFACES)
    parser.add_argument("--live", action="store_true", help="Enable physical motion on the robot")
    args = parser.parse_args()
    hardware = None
    session = None
    try:
        if args.live:
            if not sys.stdin.isatty():
                parser.error("Live testing requires an interactive terminal.")
            print("The ARX SDK may move hardware while initializing.")
            print("Clear the motion area and have someone at the physical stop.")
            if input(f"Type LIVE {args.mode} to initialize: ") != f"LIVE {args.mode}":
                return 1
            signal.signal(signal.SIGHUP, interrupt)
            signal.signal(signal.SIGTERM, interrupt)
            hardware = Hardware(args.mode)
        else:
            print("Simulation. Motors are disconnected from this console.")
        session = JogSession(args.mode, hardware)
        console(session)
    except KeyboardInterrupt:
        return 0
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f"Test ended: {exc}", file=sys.stderr)
        return 1
    finally:
        if session:
            session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

