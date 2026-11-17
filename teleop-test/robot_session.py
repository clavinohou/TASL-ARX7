"""Whole-robot startup, feedback checks, and bounded manual control."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import sys
import time

from can_monitor import FILTER, FRAME, SCALE
from jog import (
    ARM_RADIUS_METERS, ARM_STEP_METERS, BODY_FEEDBACK_TIMEOUT,
    BRAKE_SECONDS, DRIVE, INTERFACES, ZERO, Hardware, checked_pose,
)

SIDES = ("left", "right")
ARM_KEYS = {"i": (2, 1), "k": (2, -1), "j": (1, 1), "l": (1, -1),
            "u": (0, 1), "o": (0, -1)}
BODY_KEYS = {"r": (0, 0.10), "f": (0, -0.10), "q": (1, 0.02), "e": (1, -0.02),
             "1": (3, 0.02), "2": (3, -0.02)}
GRIPPER_KEYS = {"g": 5.0, "h": 0.0}
JOG_KEYS = ARM_KEYS.keys() | BODY_KEYS.keys() | GRIPPER_KEYS.keys()
DRIVE_KEYS = {key: DRIVE[key] for key in "wasd"}
DRIVE_KEYS.update(left=DRIVE["e"], right=DRIVE["r"])

HOME_TIMEOUT = 30.0
REST_TIMEOUT = 6.0
JOG_TIMEOUT = 2.0
SAMPLE_INTERVAL = 0.025
SETTLE_TIME = 0.35
ARM_JOINTS = range(1, 8)
ARM_TOLERANCE = 0.003
BODY_TOLERANCES = {0: 0.04, 1: 0.012, 3: 0.012}
BODY_LIMITS = {0: (20.0, 0.50), 1: (3.14, 0.12), 3: (0.8, 0.08)}
BODY_MOTORS = {0: 1, 1: 4, 3: 6}
BODY_SPEED_LIMIT = 0.20
REST_SPREAD = 0.012
REST_RISE = 0.006
REST_LIFT = 0.10


def drive_velocity(keys):
    x, y, yaw = (sum(components) for components in zip(*(DRIVE_KEYS[key] for key in keys), ZERO))
    scale = max(1.0, math.hypot(x, y) / DRIVE["w"][0])
    return (x / scale, y / scale, yaw)


def probe_sdk():
    project = Path(__file__).resolve().parent
    sys.path.insert(0, str(project / "robot-original" / "myx7s"))
    from bimanual import SingleArm
    import lift_controller  # initializes the vendor extension search path
    import arx_lift_python

    if not callable(SingleArm) or not callable(arx_lift_python.LiftHeadControlLoop):
        raise RuntimeError("ARX controller classes are unavailable")


def can_socket(interface, *identifiers):
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        mask = socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG | socket.CAN_SFF_MASK
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                       b"".join(FILTER.pack(ident, mask) for ident in identifiers))
        bus.bind((interface,))
        bus.setblocking(False)
    except OSError:
        bus.close()
        raise
    return bus


def incoming(bus):
    for _ in range(256):
        try:
            frame, _, flags, _ = bus.recvmsg(FRAME.size)
        except BlockingIOError:
            return
        ident, length, data = FRAME.unpack(frame)
        if length == 8 and not flags & socket.MSG_DONTROUTE:
            yield ident, data


@dataclass
class ArmFeedback:
    pose: tuple
    joints: tuple
    speeds: tuple

    def quiet(self):
        return all(abs(speed) < 0.10 for speed in self.speeds)

    def home(self):
        return self.quiet() and all(abs(joint) < 0.08 for joint in self.joints)

    def at_pose(self, target):
        orientation = abs(sum(a * b for a, b in zip(self.pose[3:], target[3:])))
        orientation /= math.hypot(*self.pose[3:]) * math.hypot(*target[3:])
        return (self.quiet() and math.dist(self.pose[:3], target[:3]) <= ARM_TOLERANCE
                and orientation >= math.cos(0.08 / 2))


class ArmWorker:
    """Keep the native arm constructor and its control thread off the body loop."""

    def __init__(self, side):
        read_fd, write_fd = os.pipe()
        try:
            os.set_blocking(read_fd, False)
            self.process = subprocess.Popen(
                [sys.executable, "-sBu", __file__, side, str(write_fd)],
                stdin=subprocess.PIPE, stdout=sys.stderr, bufsize=0, pass_fds=(write_fd,),
            )
        except OSError:
            os.close(read_fd)
            raise
        finally:
            os.close(write_fd)
        self.fd = read_fd
        os.set_blocking(self.process.stdin.fileno(), False)
        self.pending = b""
        self.latest = None
        self.error = None

    def observe(self):
        try:
            data = os.read(self.fd, 8192)
        except BlockingIOError:
            return
        if not data:
            self.error = self.error or "SDK process ended"
            self.latest = None
            return
        frames = (self.pending + data).split(b"\n")
        self.pending = frames.pop()
        for frame in frames:
            message = json.loads(frame)
            if "error" in message:
                self.error = message["error"]
                self.latest = None
            else:
                self.latest = (message["sampled"], ArmFeedback(tuple(message["pose"]), tuple(message["joints"]),
                                                tuple(message["speeds"])))

    def send(self, op, value=None):
        self.process.stdin.write(json.dumps([op, value], allow_nan=False).encode() + b"\n")

    def close(self):
        try:
            self.process.stdin.close()
            self.process.wait(timeout=0.6)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        finally:
            os.close(self.fd)


def arm_worker(side, output):
    hardware = None
    os.set_blocking(output, False)

    def report(message):
        try:
            os.write(output, json.dumps(message, allow_nan=False).encode() + b"\n")
        except BlockingIOError:
            pass

    def joints(method):
        values = tuple(method())
        if len(values) < 7 or not all(math.isfinite(value) for value in values[:7]):
            raise ValueError("Invalid joint feedback")
        return values[:7]

    try:
        hardware = Hardware(side)
        arm = hardware.arm
        pending = b""
        next_report = 0.0
        while True:
            if select.select([sys.stdin], [], [], SAMPLE_INTERVAL)[0]:
                data = os.read(sys.stdin.fileno(), 4096)
                if not data:
                    return
                frames = (pending + data).split(b"\n")
                pending = frames.pop()
                for frame in frames:
                    op, value = json.loads(frame)
                    if op == "home":
                        arm.go_home()
                    elif op == "move":
                        hardware.move_arm(checked_pose(value))
                    elif op == "hold":
                        arm.set_joint_positions(value)
                    elif op == "protect":
                        arm.protect_mode()
                    elif op == "grip":
                        arm.set_catch_pos(value)
            now = time.monotonic()
            if now >= next_report:
                report({"sampled": now, "pose": hardware.arm_pose(), "joints": joints(arm.get_joint_positions),
                        "speeds": joints(arm.get_joint_velocities)})
                next_report = now + SAMPLE_INTERVAL
    except (OSError, RuntimeError, ValueError) as exc:
        try:
            report({"error": str(exc)})
        except BrokenPipeError:
            pass
    finally:
        if hardware is not None:
            hardware.arm.protect_mode()


class RobotHardware:
    def __init__(self):
        self.arms = {}
        self.body = None
        self.sockets = {}
        self.motor_times = {side: {} for side in (*SIDES, "base")}
        self.body_speeds = {}
        self.wheels = None
        self.next_brake = 0.0
        down = []
        for name in (*SIDES, "base"):
            interface = INTERFACES[name]
            flags = Path("/sys/class/net") / interface / "flags"
            if not flags.is_file() or not int(flags.read_text(), 16) & 1:
                down.append(interface)
        if down:
            raise RuntimeError(f"Bring up {', '.join(down)} before enabling the robot.")
        try:
            for name in (*SIDES, "base"):
                identifiers = (0, 0x702) if name == "base" else (0,)
                self.sockets[name] = can_socket(INTERFACES[name], *identifiers)
        except OSError:
            self.close()
            raise

    def open(self, name):
        if name == "base":
            self.body = Hardware(name)
        else:
            self.arms[name] = ArmWorker(name)

    def observe(self, now):
        for arm in self.arms.values():
            arm.observe()
        for name, bus in self.sockets.items():
            times = self.motor_times[name]
            for ident, data in incoming(bus):
                if ident == 0x702:
                    self.wheels = (now, tuple(value * SCALE for value in struct.unpack(">4h", data)))
                else:
                    motor = data[0] & 0x0f
                    if data[0] >> 4 == 1:
                        times[motor] = now
                        if name == "base":
                            velocity = (data[3] << 4) | (data[4] >> 4)
                            self.body_speeds[motor] = velocity * 90 / 4095 - 45
                    else:
                        times.pop(motor, None)

    def feedback(self, side, now):
        arm = self.arms.get(side)
        if arm is None or arm.latest is None:
            return None
        received, feedback = arm.latest
        times = self.motor_times[side]
        if now - received >= BODY_FEEDBACK_TIMEOUT or any(
            now - times.get(motor, -math.inf) >= BODY_FEEDBACK_TIMEOUT for motor in ARM_JOINTS
        ):
            return None
        return feedback

    def faults(self):
        return [f"{side} arm: {arm.error}" for side, arm in self.arms.items() if arm.error]

    def home(self):
        for side in SIDES:
            self.arms[side].send("home")

    def move_arm(self, side, target):
        self.arms[side].send("move", target)

    def hold_arm(self, side, feedback):
        arm = self.arms[side]
        if arm.latest is None and arm.error is None:
            # A native constructor can still be homing and cannot read a pipe yet.
            arm.process.terminate()
        elif feedback is None:
            arm.send("protect")
        else:
            arm.send("hold", feedback.joints)

    def grip(self, target):
        for side in SIDES:
            self.arms[side].send("grip", target)

    def body_feedback(self, now):
        if self.body is None:
            return {}
        self.body.post_zero_positions(now)
        calibration = self.body.calibration
        if calibration.disabled:
            raise RuntimeError("The SDK disabled the body motors.")
        self.body.body.read()
        position = self.body.body_position()
        enabled = {axis for axis in (0, 1) if BODY_MOTORS[axis] in calibration.holding
                   and calibration.fresh(BODY_MOTORS[axis], now)}
        if now - self.motor_times["base"].get(BODY_MOTORS[3], -math.inf) < BODY_FEEDBACK_TIMEOUT:
            enabled.add(3)
        return {axis: position[axis] for axis in enabled}

    def body_quiet(self, axis):
        return abs(self.body_speeds.get(BODY_MOTORS[axis], math.inf)) < BODY_SPEED_LIMIT

    def body_tick(self, targets, velocity, calibrating):
        if self.body is not None and (calibrating or 0 in targets and 1 in targets):
            position = (targets.get(0, 0.0), targets.get(1, 0.0), 0.0, 0.0)
            self.body.body_tick(position, velocity, velocity != ZERO, targets.get(3))
        elif self.arms and time.monotonic() >= self.next_brake:
            self.brake()

    def brake(self):
        bus = self.sockets.get("base")
        if bus is not None:
            # Same mode-2 frame as the SDK. This also works before body initialization.
            bus.send(FRAME.pack(0x701, 8, struct.pack(">4h", 0, 0, 2, 0)))
            self.next_brake = time.monotonic() + 0.05

    def wheel_speeds(self, now):
        if self.wheels and now - self.wheels[0] < 0.50:
            return self.wheels[1]
        return None

    def close(self):
        if self.body is not None:
            self.body.calibration.bus.close()
            self.body = None
        for arm in self.arms.values():
            arm.close()
        self.arms.clear()
        for bus in self.sockets.values():
            bus.close()
        self.sockets.clear()


class Simulator:
    """Synthetic feedback with separate arm and body arrival times."""

    def __init__(self):
        self.arms = {}
        self.body = False
        self.poses = {side: (0.32, y, 0.22, 1.0, 0.0, 0.0, 0.0)
                      for side, y in zip(SIDES, (0.20, -0.20))}
        self.targets = self.poses.copy()
        self.position = {0: 0.0, 1: 0.0, 3: 0.30}
        self.now = 0.0
        self.homing_at = math.inf
        self.body_at = math.inf
        self.velocity = ZERO

    def open(self, name):
        if name == "base":
            self.body = True
            self.body_at = self.now
        else:
            self.arms[name] = True

    def observe(self, now):
        self.now = now
        for side in self.arms:
            self.poses[side] = tuple(value + 0.25 * (goal - value)
                                     for value, goal in zip(self.poses[side], self.targets[side]))

    def feedback(self, side, now):
        if side not in self.arms:
            return None
        duration = 0.55 if side == "left" else 0.90
        joint = 0.24 if now - self.homing_at < duration else 0.0
        return ArmFeedback(self.poses[side], (joint,) * 7, (0.0,) * 7)

    def faults(self):
        return []

    def home(self):
        self.homing_at = self.now

    def move_arm(self, side, target):
        self.targets[side] = target

    def hold_arm(self, side, feedback):
        self.targets[side] = self.poses[side]

    def grip(self, target):
        pass

    def body_feedback(self, now):
        delay = {0: 0.70, 1: 1.30, 3: 0.30}
        return {axis: value for axis, value in self.position.items()
                if now - self.body_at >= delay[axis]}

    def body_quiet(self, axis):
        return True

    def body_tick(self, targets, velocity, calibrating):
        if self.body:
            for axis, target in targets.items():
                self.position[axis] += 0.25 * (target - self.position[axis])
            self.velocity = velocity

    def brake(self):
        self.velocity = ZERO

    def wheel_speeds(self, now):
        if self.body:
            x, y, yaw = self.velocity
            return tuple(10 * value for value in (x - y - yaw, x + y + yaw, x + y - yaw, x - y + yaw))
        return None

    def close(self):
        pass


class RobotSession:
    def __init__(self, rig, now):
        self.rig = rig
        self.phase = "homing"
        self.notice = "Opening the arm and body controllers."
        self.deadline = now + HOME_TIMEOUT
        self.next_sample = now
        self.can_start_homing = False
        self.opened = set()
        self.home_sent = False
        self.settling = {}
        self.arm_state = {}
        self.body_state = {}
        self.last_body_position = {}
        self.arm_targets = {}
        self.body_targets = {}
        self.arm_rest = {}
        self.body_rest = {}
        self.arm_deadline = None
        self.body_deadlines = {}
        self.release_required = True
        self.velocity = ZERO
        self.gripper = "—"

    def settled(self, name, condition, now):
        if not condition:
            self.settling.pop(name, None)
            return False
        return now - self.settling.setdefault(name, now) >= SETTLE_TIME

    def advance_startup(self):
        """Open both arms together, then start body calibration on the next heartbeat."""
        if self.phase != "homing" or self.can_start_homing:
            return
        if not self.opened:
            self.rig.brake()
            for side in SIDES:
                self.rig.open(side)
                self.opened.add(side)
        elif "base" not in self.opened:
            self.rig.open("base")
            self.opened.add("base")
            self.notice = "Homing both arms and calibrating the lift and torso."
        else:
            self.can_start_homing = True

    def brake(self):
        self.velocity = ZERO
        if self.opened:
            self.rig.brake()

    def poll(self, now):
        self.rig.observe(now)
        self.arm_state = {side: self.rig.feedback(side, now) for side in SIDES}
        self.body_state = {}
        errors = self.rig.faults()
        if errors and self.phase != "fault":
            raise RuntimeError(", ".join(errors))
        self.body_state = self.rig.body_feedback(now)
        self.last_body_position.update(self.body_state)

    def homing(self, now):
        for axis in (0, 1):
            if axis in self.body_state and axis not in self.body_targets:
                self.body_targets[axis] = max(0.0, self.body_state[axis])
        if self.home_sent:
            for side, state in self.arm_state.items():
                self.settled(side, state is not None and state.home(), now)
        if not self.home_sent and self.can_start_homing and all(self.arm_state.values()):
            self.rig.home()
            self.home_sent = True
        if self.home_sent and all(now - self.settling.get(side, now) >= SETTLE_TIME for side in SIDES):
            # Vendor zero is the upright torso reference. Refuse a large move
            # if the reported position after SDK calibration disagrees.
            quiet = all(axis in self.body_state and self.rig.body_quiet(axis) for axis in (0, 1))
            if self.settled("body", quiet, now):
                if abs(self.body_state[0]) > 0.10 or abs(self.body_state[1]) > 0.08:
                    self.fault("Lift or torso feedback disagrees with its calibrated zero.", now)
                else:
                    self.begin_rest(now)

    def begin_rest(self, now):
        self.phase = "resting"
        self.notice = "Moving both arms out and raising the lift slightly."
        self.deadline = now + REST_TIMEOUT
        self.settling.clear()
        self.body_targets.update({0: REST_LIFT, 1: 0.0})
        self.body_rest = self.body_targets.copy()
        for side, direction in zip(SIDES, (1, -1)):
            x, y, z, *orientation = self.arm_state[side].pose
            target = (x, y + direction * REST_SPREAD, z + REST_RISE, *orientation)
            self.arm_targets[side] = target
            self.rig.move_arm(side, target)
        self.arm_rest = self.arm_targets.copy()

    def body_at(self, axis):
        return (axis in self.body_state and self.rig.body_quiet(axis)
                and abs(self.body_state[axis] - self.body_targets[axis]) <= BODY_TOLERANCES[axis])

    def arms_at(self):
        return all(self.arm_state[side] and self.arm_state[side].at_pose(target)
                   for side, target in self.arm_targets.items())

    def missing_feedback(self):
        missing = [f"{side} arm" for side in SIDES if not self.arm_state.get(side)]
        for axis, label in ((0, "lift"), (1, "torso")):
            if axis not in self.body_state:
                missing.append(label)
        if 3 in self.body_targets and 3 not in self.body_state:
            missing.append("head pitch")
        return missing

    def check_targets(self, now):
        if self.phase == "resting":
            if self.settled("rest", self.arms_at() and all(self.body_at(axis) for axis in (0, 1)), now):
                self.phase = "ready"
                self.notice = "At rest. Hold Space to use the controls."
        elif self.phase == "ready":
            if self.arm_deadline is not None:
                if self.arms_at():
                    self.arm_deadline = None
                elif now >= self.arm_deadline:
                    self.fault("Both arms did not reach the jog target.", now)
                    return
            for axis, deadline in tuple(self.body_deadlines.items()):
                if self.body_at(axis):
                    del self.body_deadlines[axis]
                elif now >= deadline:
                    self.fault("A body axis did not reach its jog target.", now)
                    return

    def tick(self, now):
        if now >= self.next_sample:
            self.next_sample = now + SAMPLE_INTERVAL
            try:
                self.poll(now)
                if self.phase == "homing":
                    self.homing(now)
                elif self.phase in ("resting", "ready", "paused"):
                    missing = self.missing_feedback()
                    if missing:
                        self.fault(f"Feedback lost: {', '.join(missing)}.", now)
                    else:
                        self.check_targets(now)
                if self.phase in ("homing", "resting") and now >= self.deadline:
                    missing = self.missing_feedback()
                    if self.phase == "homing":
                        waiting = [side + " arm" for side in SIDES
                                   if now - self.settling.get(side, now) < SETTLE_TIME]
                        pending = ", ".join(dict.fromkeys(missing + waiting))
                        detail = f"Homing timed out ({pending or 'lift or torso settling'})."
                    else:
                        detail = "The robot did not settle at the resting pose."
                    self.fault(detail, now)
            except (OSError, RuntimeError, ValueError) as exc:
                self.fault(str(exc), now)
        try:
            if self.velocity != ZERO and self.rig.wheel_speeds(now) is None:
                self.block_drive()
            self.rig.body_tick(self.body_targets, self.velocity, calibrating=self.phase == "homing")
        except (OSError, RuntimeError, ValueError) as exc:
            if self.phase != "fault":
                self.fault(f"Body control interrupted: {exc}", now)

    def freeze(self, now, all_arms=False):
        try:
            self.rig.observe(now)
            body = self.rig.body_feedback(now)
        except (OSError, RuntimeError, ValueError):
            body = {}
        errors = []
        if all_arms or self.arm_deadline is not None:
            for side in SIDES:
                if side not in self.opened:
                    continue
                try:
                    feedback = self.rig.feedback(side, now)
                    self.rig.hold_arm(side, feedback)
                    if feedback:
                        self.arm_targets[side] = feedback.pose
                    else:
                        errors.append(f"{side} arm hold unavailable")
                except (OSError, RuntimeError, ValueError):
                    try:
                        self.rig.hold_arm(side, None)
                    except (OSError, RuntimeError, ValueError):
                        pass
                    errors.append(f"{side} arm hold unavailable")
            self.arm_deadline = None
        axes = self.body_targets if all_arms else self.body_deadlines
        for axis in axes:
            if axis not in body:
                errors.append("body axis hold unconfirmed")
            position = body.get(axis, self.last_body_position.get(axis))
            if position is not None:
                self.body_targets[axis] = max(0.0, min(BODY_LIMITS[axis][0], position))
        self.body_deadlines.clear()
        return errors

    def stop(self, notice, now):
        self.velocity = ZERO
        self.release_required = True
        if self.phase == "fault":
            return
        if self.phase in ("homing", "resting"):
            self.fault("Startup interrupted.", now)
        else:
            try:
                self.brake()
                errors = self.freeze(now)
            except (OSError, RuntimeError, ValueError) as exc:
                self.fault(f"Stop interrupted: {exc}", now)
                return
            if errors:
                self.fault(", ".join(errors) + ".", now)
            else:
                self.phase = "paused"
                self.notice = notice

    def fault(self, detail, now):
        if self.phase == "fault":
            return
        self.phase = "fault"
        self.notice = f"{detail} Use the physical stop if needed. Support the robot before reconnecting."
        errors = []
        try:
            self.brake()
        except OSError:
            errors.append("chassis brake unconfirmed")
        try:
            errors.extend(self.freeze(now, all_arms=True))
        except (OSError, RuntimeError, ValueError):
            errors.append("actuator hold unconfirmed")
        if not {0, 1} <= self.body_targets.keys() and "base" in self.opened:
            errors.append("body hold unconfirmed")
        if errors:
            self.notice = f"{detail} {', '.join(errors).capitalize()}. Use the physical stop if needed."

    def resume(self):
        if self.phase == "paused" and not self.missing_feedback():
            self.phase = "ready"
            self.notice = "Enabled. Hold Space to use the controls."

    def block_drive(self):
        self.brake()
        self.release_required = True
        self.notice = "Wheel feedback unavailable. Release Space before retrying."

    def input(self, deadman, keys, action, now):
        if self.phase != "ready":
            return
        if not deadman:
            self.release_required = False
            if self.velocity != ZERO:
                self.brake()
            if self.arm_deadline is not None or self.body_deadlines:
                errors = self.freeze(now)
                if errors:
                    self.fault(", ".join(errors) + ".", now)
            return
        if self.release_required:
            return
        velocity = drive_velocity(keys)
        if velocity != ZERO and self.rig.wheel_speeds(now) is None:
            self.block_drive()
            return
        self.velocity = velocity
        if action in ARM_KEYS:
            self.jog_arms(action, now)
        elif action in BODY_KEYS:
            self.jog_body(action, now)
        elif action in GRIPPER_KEYS:
            self.rig.grip(GRIPPER_KEYS[action])
            self.gripper = "Open requested" if action == "g" else "Close requested"
            self.notice = f"Both grippers: {self.gripper.lower()}."

    def jog_arms(self, action, now):
        if self.arm_deadline is not None:
            self.notice = "Wait for both arms to finish their current step."
            return
        axis, direction = ARM_KEYS[action]
        targets = {}
        for side, measured in self.arm_state.items():
            # Same XYZ offset for each arm, from its own target.
            pose = list(self.arm_targets.get(side, measured.pose))
            pose[axis] += direction * ARM_STEP_METERS
            if math.dist(pose[:3], self.arm_rest[side][:3]) > ARM_RADIUS_METERS + 1e-9:
                self.notice = "Arm jog limit reached."
                return
            targets[side] = tuple(pose)
        self.arm_deadline = now + JOG_TIMEOUT
        for side, pose in targets.items():
            self.rig.move_arm(side, pose)
        self.arm_targets = targets
        self.notice = f"Both arms jogged {'XYZ'[axis]}{'+' if direction > 0 else '−'}."

    def jog_body(self, action, now):
        axis, step = BODY_KEYS[action]
        if axis not in self.body_state:
            self.notice = "That axis has no fresh feedback."
            return
        if axis in self.body_deadlines:
            self.notice = "Wait for the axis to finish its current step."
            return
        actual = self.body_state[axis]
        maximum, travel = BODY_LIMITS[axis]
        if not -BODY_TOLERANCES[axis] <= actual <= maximum:
            self.notice = "The axis is outside its jog range."
            return
        if axis == 3:
            self.body_rest.setdefault(axis, max(0.0, actual))
        goal = round(self.body_targets.get(axis, max(0.0, actual)) + step, 6)
        if axis in (0, 1) and step < 0:
            goal = max(0.0, goal)
        if not 0.0 <= goal <= maximum or abs(goal - self.body_rest[axis]) > travel + 1e-9:
            self.notice = "Body jog limit reached."
            return
        self.body_targets[axis] = goal
        self.body_deadlines[axis] = now + JOG_TIMEOUT
        name = {0: "Lift", 1: "Torso pivot", 3: "Head pitch"}[axis]
        self.notice = f"{name} target {goal:.2f}."

    def status(self, now):
        if self.phase == "homing":
            arms = {side: ("Homed" if now - self.settling.get(side, now) >= SETTLE_TIME else
                           "Homing" if self.home_sent else
                           "Waiting for encoders" if side in self.opened else "Waiting")
                    for side in SIDES}
        else:
            arms = {side: ("Feedback lost" if not self.arm_state.get(side) else
                           {"ready": "Ready", "resting": "Moving to rest",
                            "paused": "Paused", "fault": "Motion stopped"}[self.phase]) for side in SIDES}
        return {
            "phase": self.phase, "notice": self.notice, "arms": arms,
            "poses": {side: state.pose[:3] for side, state in self.arm_state.items() if state},
            "body": self.body_state, "velocity": self.velocity,
            "wheels": self.rig.wheel_speeds(now), "gripper": self.gripper,
            "started": bool(self.opened),
            "body_moving": sorted(self.body_deadlines),
        }

    def close(self):
        try:
            if self.opened:
                self.stop("Disconnected.", time.monotonic())
                self.brake()
                deadline = time.monotonic() + BRAKE_SECONDS
                while time.monotonic() < deadline:
                    self.rig.body_tick(self.body_targets, ZERO, calibrating=False)
                    time.sleep(0.01)
        finally:
            self.rig.close()


if __name__ == "__main__":
    arm_worker(sys.argv[1], int(sys.argv[2]))
