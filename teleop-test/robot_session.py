"""Whole-robot startup, feedback checks, parking, and bounded manual control."""

from __future__ import annotations

from collections import deque
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
    BRAKE_SECONDS, INTERFACES, LIFT_CEILING, ZERO, Hardware, checked_pose,
)

SIDES = ("left", "right")
ARM_SELECTIONS = {"both": SIDES, "left": ("left",), "right": ("right",)}
ARM_KEYS = {"i": (2, 1), "k": (2, -1), "j": (1, 1), "l": (1, -1),
            "u": (0, 1), "o": (0, -1)}
BODY_KEYS = {"r": (0, 0.10), "f": (0, -0.10), "q": (1, 0.02), "e": (1, -0.02),
             "1": (3, 0.02), "2": (3, -0.02)}
GRIPPER_KEYS = {"g": 5.0, "h": 0.0}
JOG_KEYS = ARM_KEYS.keys() | BODY_KEYS.keys() | GRIPPER_KEYS.keys()
OPERATIONS = {"enable", "park", "recover", "map"}
AXIS_NAMES = {0: "lift", 1: "torso", 3: "head"}
# Mapping mode folds both arms home (out of the head camera's view, to be confirmed
# on the live feed) with the torso upright, keeps the lift where it is, and then
# allows only driving, lift, and head pitch so nothing moves through the view.
DRIVE_PHASES = ("ready", "mapping")
MAPPING_BLOCKED = ARM_KEYS.keys() | GRIPPER_KEYS.keys() | {"q", "e"}

# Chassis. Commands are quantized to ~330 counts per m/s and ~165 counts per
# rad/s. 16 counts (0.048 m/s) did not move the base and 49 counts (0.15 m/s)
# did, so every speed level stays at or above 49 counts per active axis.
DRIVE_KEYS = {"w": (1, 0, 0), "s": (-1, 0, 0), "a": (0, 1, 0), "d": (0, -1, 0),
              "left": (0, 0, 1), "right": (0, 0, -1)}
SPEED_LEVELS = (0.15, 0.20, 0.25)  # m/s per translation axis
YAW_PER_LINEAR = 2.0               # rad/s per m/s, 0.30 rad/s at the first level
MAX_TRANSLATION = 0.30             # m/s cap on diagonal motion
LINEAR_ACCEL, LINEAR_DECEL = 0.6, 1.5  # m/s^2
YAW_ACCEL, YAW_DECEL = 1.2, 3.0        # rad/s^2
# "latched": Space holds the chassis in drive mode 1, keys only set velocity,
# releasing Space brakes (mode 2). "pulse": brake whenever no drive key is held.
DRIVE_MODES = ("latched", "pulse")
STALL_SECONDS = 1.0
STALL_COMMAND = (0.10, 0.20)  # m/s and rad/s before a stall can be judged
STALL_WHEEL = 0.02            # peak wheel report below this means no response
CREEP_SECONDS = 1.0
CREEP_WHEEL = 1.0

HOME_TIMEOUT = 30.0
REST_TIMEOUT = 6.0
PARK_TIMEOUT = 20.0
JOG_TIMEOUT = 2.0
SAMPLE_INTERVAL = 0.025
SETTLE_TIME = 0.35
ARM_JOINTS = range(1, 8)
ARM_TOLERANCE = 0.003
BODY_TOLERANCES = {0: 0.04, 1: 0.012, 3: 0.012}
BODY_LIMITS = {0: (LIFT_CEILING, 0.50), 1: (3.14, 0.12), 3: (0.8, 0.08)}
BODY_MOTORS = {0: 1, 1: 4, 3: 6}
BODY_SPEED_LIMIT = 0.20
REST_SPREAD = 0.012
REST_RISE = 0.006
REST_LIFT = 0.10
STARTUP_STEPS = (
    ("arms_open", "Arm controllers report feedback"),
    ("body_open", "Body controller opened"),
    ("left_home", "Left arm homed"),
    ("right_home", "Right arm homed"),
    ("body_cal", "Lift and torso calibrated"),
    ("rest", "Resting pose reached"),
)
TIMED_PHASES = ("homing", "resting", "parking", "stowing")
STOW_TIMEOUT = 20.0


def drive_velocity(keys, level=0):
    speed = SPEED_LEVELS[level]
    x = y = yaw = 0.0
    for key in keys:
        dx, dy, dyaw = DRIVE_KEYS[key]
        x += dx * speed
        y += dy * speed
        yaw += dyaw * speed * YAW_PER_LINEAR
    scale = max(1.0, math.hypot(x, y) / MAX_TRANSLATION)
    return (x / scale, y / scale, yaw)


def ramp(current, target, dt):
    """Limit acceleration. Slowing down and reversing use the faster rate."""
    limits = ((LINEAR_ACCEL, LINEAR_DECEL), (LINEAR_ACCEL, LINEAR_DECEL), (YAW_ACCEL, YAW_DECEL))
    result = []
    for value, goal, (accel, decel) in zip(current, target, limits):
        speeding_up = goal * value >= 0 and abs(goal) > abs(value)
        step = (accel if speeding_up else decel) * dt
        if abs(goal - value) <= step:
            result.append(goal)
        else:
            result.append(value + math.copysign(step, goal - value))
    return tuple(result)


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

    def grip(self, target, sides=SIDES):
        for side in sides:
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

    def body_tick(self, targets, velocity, armed, calibrating):
        if self.body is not None and (calibrating or 0 in targets and 1 in targets):
            position = (targets.get(0, 0.0), targets.get(1, 0.0), 0.0, 0.0)
            self.body.body_tick(position, velocity if armed else ZERO, 1 if armed else 2, targets.get(3))
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
    """Synthetic feedback with separate arm and body arrival times.

    `stall` keeps the wheels at zero to rehearse the no-response warning.
    `withheld` names feedback to drop ("left", "right", "lift", "torso") for tests.
    """

    HOME = {"left": (0.32, 0.20, 0.22, 1.0, 0.0, 0.0, 0.0),
            "right": (0.32, -0.20, 0.22, 1.0, 0.0, 0.0, 0.0)}
    HOMING_SECONDS = {"left": 0.55, "right": 0.90}
    BODY_DELAY = {0: 0.70, 1: 1.30, 3: 0.30}

    def __init__(self, stall=False):
        self.arms = set()
        self.body = False
        self.poses = dict(self.HOME)
        self.targets = dict(self.HOME)
        self.position = {0: 0.0, 1: 0.0, 3: 0.30}
        self.body_targets = {}
        self.now = 0.0
        self.homing_at = -math.inf
        self.body_at = math.inf
        self.velocity = ZERO
        self.armed = False
        self.stall = stall
        self.withheld = set()
        self.errors = []
        self.grips = {}
        self.modes = []

    def open(self, name):
        if name == "base":
            self.body = True
            self.body_at = self.now
        else:
            self.arms.add(name)

    def observe(self, now):
        self.now = now
        for side in self.arms:
            self.poses[side] = tuple(value + 0.25 * (goal - value)
                                     for value, goal in zip(self.poses[side], self.targets[side]))
        if self.body:
            for axis, target in self.body_targets.items():
                self.position[axis] += 0.25 * (target - self.position[axis])

    def feedback(self, side, now):
        if side not in self.arms or side in self.withheld:
            return None
        if now - self.homing_at < self.HOMING_SECONDS[side]:
            joint = 0.24
        else:
            joint = min(1.0, 10 * math.dist(self.poses[side][:3], self.HOME[side][:3]))
        return ArmFeedback(self.poses[side], (joint,) * 7, (0.0,) * 7)

    def faults(self):
        return list(self.errors)

    def home(self):
        self.homing_at = self.now
        for side in self.arms:
            self.targets[side] = self.HOME[side]

    def move_arm(self, side, target):
        self.targets[side] = tuple(target)

    def hold_arm(self, side, feedback):
        self.targets[side] = self.poses[side]

    def grip(self, target, sides=SIDES):
        for side in sides:
            self.grips[side] = target

    def body_feedback(self, now):
        if not self.body:
            return {}
        names = {0: "lift", 1: "torso", 3: "head"}
        return {axis: value for axis, value in self.position.items()
                if now - self.body_at >= self.BODY_DELAY[axis] and names[axis] not in self.withheld}

    def body_quiet(self, axis):
        return True

    def body_tick(self, targets, velocity, armed, calibrating):
        if self.body:
            self.body_targets = dict(targets)
            self.armed = armed
            self.velocity = velocity if armed else ZERO
            mode = 1 if armed else 2
            if not self.modes or self.modes[-1][0] != mode or self.modes[-1][1] != self.velocity:
                self.modes.append((mode, self.velocity))
                del self.modes[:-200]

    def brake(self):
        self.velocity = ZERO
        self.armed = False

    def wheel_speeds(self, now):
        if not self.body:
            return None
        if self.stall:
            return (0.0,) * 4
        x, y, yaw = self.velocity
        return tuple(10 * value for value in (x - y - yaw, x + y + yaw, x + y - yaw, x - y + yaw))

    def close(self):
        pass


class RobotSession:
    def __init__(self, rig, now):
        self.rig = rig
        self.phase = "homing"
        self.notice = ""
        self.events = deque(maxlen=40)
        self.event_seq = 0
        self.last_event = (None, -math.inf)
        self.deadline = now + HOME_TIMEOUT
        self.next_sample = now
        self.last_tick = now
        self.can_start_homing = False
        self.opened = set()
        self.home_sent = False
        self.settling = {}
        self.done = set()
        self.startup_complete = False
        self.arm_state = {}
        self.body_state = {}
        self.last_body_position = {}
        self.arm_targets = {}
        self.body_targets = {}
        self.arm_rest = {}
        self.body_rest = {}
        self.arm_deadline = None
        self.arm_moving = set()
        self.body_deadlines = {}
        self.release_required = True
        self.armed = False
        self.velocity = ZERO
        self.command = ZERO
        self.speed_level = 0
        self.selection = "both"
        self.drive_mode = DRIVE_MODES[0]
        self.drive_since = None
        self.drive_peak = 0.0
        self.creep_since = None
        self.wheel_warning = None
        self.wheels_proven = False
        self.gripper = {side: "—" for side in SIDES}
        self.fault_detail = None
        self.resume_phase = "ready"
        self.say("Opening the arm and body controllers.")

    # Event log ------------------------------------------------------------

    def say(self, text, level="info"):
        self.notice = text
        now = time.monotonic()
        previous, when = self.last_event
        if text == previous and now - when < 1.0:
            return
        self.last_event = (text, now)
        self.event_seq += 1
        self.events.append([self.event_seq, level, text])

    def settled(self, name, condition, now):
        if not condition:
            self.settling.pop(name, None)
            return False
        return now - self.settling.setdefault(name, now) >= SETTLE_TIME

    # Startup ----------------------------------------------------------------

    def advance_startup(self):
        """Open both arms together, then start body calibration on the next heartbeat."""
        if self.phase != "homing" or self.can_start_homing:
            return
        if not {"left", "right"} <= self.opened:
            self.rig.brake()
            for side in SIDES:
                if side not in self.opened:
                    self.rig.open(side)
                    self.opened.add(side)
        elif "base" not in self.opened:
            self.rig.open("base")
            self.opened.add("base")
            self.done.add("body_open")
            self.say("Homing both arms and calibrating the lift and torso.")
        else:
            self.can_start_homing = True

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
        if all(self.arm_state.values()):
            self.done.add("arms_open")
        if 0 in self.body_state and 1 in self.body_state:
            self.done.add("body_cal")
        for axis in (0, 1):
            if axis in self.body_state and axis not in self.body_targets:
                self.body_targets[axis] = max(0.0, self.body_state[axis])
        if self.home_sent:
            for side, state in self.arm_state.items():
                if self.settled(side, state is not None and state.home(), now):
                    self.done.add(f"{side}_home")
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
        self.resume_phase = "ready"
        self.say("Moving the arms out and raising the lift slightly.")
        self.deadline = now + REST_TIMEOUT
        self.settling.clear()
        self.done.update(("arms_open", "body_open", "left_home", "right_home", "body_cal"))
        self.body_targets.update({0: REST_LIFT, 1: 0.0})
        self.body_rest = self.body_targets.copy()
        for side, direction in zip(SIDES, (1, -1)):
            x, y, z, *orientation = self.arm_state[side].pose
            target = (x, y + direction * REST_SPREAD, z + REST_RISE, *orientation)
            self.arm_targets[side] = target
            self.rig.move_arm(side, target)
        self.arm_rest = self.arm_targets.copy()

    def begin_park(self, now):
        if self.phase not in ("ready", "paused", "mapping"):
            return
        self.resume_phase = "ready"
        self.brake()
        self.release_required = True
        self.arm_deadline = None
        self.arm_moving.clear()
        self.body_deadlines.clear()
        self.rig.home()
        self.arm_targets = {}
        self.body_targets.update({0: 0.0, 1: 0.0})
        self.phase = "parking"
        self.deadline = now + PARK_TIMEOUT
        self.settling.clear()
        self.say("Parking: arms to home, torso upright, lift down.")

    def begin_map(self, now):
        """Fold both arms home and hold the torso upright. The lift stays at the camera height."""
        if self.phase not in ("ready", "paused"):
            return
        self.resume_phase = "ready"
        self.brake()
        self.release_required = True
        self.arm_deadline = None
        self.arm_moving.clear()
        self.body_deadlines.clear()
        self.rig.home()
        self.arm_targets = {}
        self.body_targets[1] = 0.0
        self.phase = "stowing"
        self.deadline = now + STOW_TIMEOUT
        self.settling.clear()
        self.say("Mapping mode: folding both arms home and holding the torso upright.")

    # Target checks ------------------------------------------------------------

    def body_at(self, axis):
        return (axis in self.body_state and self.rig.body_quiet(axis)
                and abs(self.body_state[axis] - self.body_targets[axis]) <= BODY_TOLERANCES[axis])

    def arms_at(self, sides=SIDES):
        return all(self.arm_state.get(side) and self.arm_state[side].at_pose(self.arm_targets[side])
                   for side in sides if side in self.arm_targets)

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
                self.done.add("rest")
                self.startup_complete = True
                self.say("At rest. Hold Space to use the controls.")
        elif self.phase == "parking":
            home = all(self.arm_state[side].home() for side in SIDES)
            if self.settled("park", home and all(self.body_at(axis) for axis in (0, 1)), now):
                self.phase = "parked"
                self.say("Parked: arms home, torso upright, lift down. Support the robot, then disconnect, "
                         "or press Enable to return to the resting pose.")
        elif self.phase == "stowing":
            home = all(self.arm_state[side].home() for side in SIDES)
            if self.settled("stow", home and self.body_at(1), now):
                self.phase = "mapping"
                self.resume_phase = "mapping"
                self.say("Arms folded for mapping. Hold Space to drive. Arm, gripper, and torso keys are off.")
        elif self.phase in DRIVE_PHASES:
            if self.arm_deadline is not None:
                if self.arms_at(self.arm_moving):
                    self.arm_deadline = None
                    self.arm_moving.clear()
                elif now >= self.arm_deadline:
                    self.fault("An arm did not reach its jog target.", now)
                    return
            for axis, deadline in tuple(self.body_deadlines.items()):
                if self.body_at(axis):
                    del self.body_deadlines[axis]
                elif now >= deadline:
                    self.fault(f"The {AXIS_NAMES[axis]} did not reach its jog target.", now)
                    return

    # Control loop ---------------------------------------------------------------

    def tick(self, now):
        dt = min(0.05, max(0.0, now - self.last_tick))
        self.last_tick = now
        if now >= self.next_sample:
            self.next_sample = now + SAMPLE_INTERVAL
            try:
                self.poll(now)
                if self.phase == "homing":
                    self.homing(now)
                elif self.phase in ("resting", "ready", "paused", "parking", "parked", "stowing", "mapping"):
                    missing = self.missing_feedback()
                    if missing:
                        self.fault(f"Feedback lost: {', '.join(missing)}.", now)
                    else:
                        self.check_targets(now)
                if self.phase in TIMED_PHASES and now >= self.deadline:
                    if self.phase == "homing":
                        waiting = [side + " arm" for side in SIDES
                                   if now - self.settling.get(side, now) < SETTLE_TIME]
                        pending = ", ".join(dict.fromkeys(self.missing_feedback() + waiting))
                        detail = f"Homing timed out ({pending or 'lift or torso settling'})."
                    elif self.phase == "resting":
                        detail = "The robot did not settle at the resting pose."
                    elif self.phase == "stowing":
                        detail = "The arms did not fold for mapping in time."
                    else:
                        detail = "Parking did not finish in time."
                    self.fault(detail, now)
            except (OSError, RuntimeError, ValueError) as exc:
                self.fault(str(exc), now)
        try:
            if self.armed and self.rig.wheel_speeds(now) is None:
                self.block_drive("Wheel feedback lost. Chassis braked. Release Space before retrying.")
            self.command = ramp(self.command, self.velocity, dt) if self.armed else ZERO
            self.check_wheels(now)
            self.rig.body_tick(self.body_targets, self.command, self.armed, calibrating=self.phase == "homing")
        except (OSError, RuntimeError, ValueError) as exc:
            if self.phase != "fault":
                self.fault(f"Body control interrupted: {exc}", now)

    def check_wheels(self, now):
        wheels = self.rig.wheel_speeds(now)
        if wheels is None:
            self.drive_since = self.creep_since = None
            return
        peak = max(abs(value) for value in wheels)
        x, y, yaw = self.command
        if self.armed and (math.hypot(x, y) >= STALL_COMMAND[0] or abs(yaw) >= STALL_COMMAND[1]):
            self.creep_since = None
            if self.drive_since is None:
                self.drive_since, self.drive_peak = now, 0.0
            self.drive_peak = max(self.drive_peak, peak)
            if self.drive_peak >= STALL_WHEEL:
                self.wheels_proven = True
                if self.wheel_warning == "No wheel response":
                    self.wheel_warning = None
            if now - self.drive_since >= STALL_SECONDS and self.drive_peak < STALL_WHEEL:
                if self.wheels_proven:
                    self.wheel_warning = "No wheel response"
                    self.block_drive("The wheels reported no motion during a 1 s drive command. Chassis braked. "
                                     "Check for an obstruction, the chassis power, and the unlock.", "warn")
                elif self.wheel_warning != "No wheel response":
                    # Until the wheel reports have shown motion once, they cannot be trusted
                    # to stop a drive. Warn so the operator can compare with what they see.
                    self.wheel_warning = "No wheel response"
                    self.say("The wheels report no motion during this drive command. If the base is moving, "
                             "the wheel feedback is not working; if it is not, check the chassis power and unlock.",
                             "warn")
            return
        self.drive_since = None
        if not self.armed and peak > CREEP_WHEEL:
            if self.creep_since is None:
                self.creep_since = now
            elif now - self.creep_since >= CREEP_SECONDS and self.wheel_warning != "Moving while braked":
                self.wheel_warning = "Moving while braked"
                self.say("The wheels report motion while the chassis is braked. Check whether the robot "
                         "is being pushed. Use the physical stop if it is driving itself.", "warn")
        else:
            self.creep_since = None
            if self.wheel_warning == "Moving while braked":
                self.wheel_warning = None

    def brake(self):
        self.armed = False
        self.velocity = ZERO
        self.command = ZERO
        if self.opened:
            self.rig.brake()

    def block_drive(self, text, level="warn"):
        self.brake()
        self.release_required = True
        self.say(text, level)

    def cut_drive(self):
        """Input arrived late. Brake the chassis but keep holding the arms and body."""
        if self.armed or self.velocity != ZERO:
            self.block_drive("Input delayed. Chassis braked. Release Space before retrying.")

    def freeze(self, now, all_arms=False):
        try:
            self.rig.observe(now)
            body = self.rig.body_feedback(now)
        except (OSError, RuntimeError, ValueError):
            body = {}
        errors = []
        if all_arms or self.arm_deadline is not None:
            for side in (SIDES if all_arms else sorted(self.arm_moving)):
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
            self.arm_moving.clear()
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
        self.release_required = True
        if self.phase == "fault":
            self.brake()
            return
        if self.phase in ("homing", "resting"):
            self.fault("Startup interrupted.", now)
        elif self.phase == "parked":
            self.brake()
            self.say(notice)
        else:
            try:
                self.brake()
                errors = self.freeze(now, all_arms=self.phase in ("parking", "stowing"))
            except (OSError, RuntimeError, ValueError) as exc:
                self.fault(f"Stop interrupted: {exc}", now)
                return
            if errors:
                self.fault(", ".join(errors).capitalize() + ".", now)
            else:
                self.phase = "paused"
                self.say(notice)

    def fault(self, detail, now):
        if self.phase == "fault":
            return
        self.phase = "fault"
        self.fault_detail = detail
        self.resume_phase = "ready"
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
        text = f"{detail} Motion locked and holding."
        if errors:
            text = f"{detail} {', '.join(dict.fromkeys(errors)).capitalize()}. Use the physical stop if needed."
        self.say(text, "fault")

    def recovery_blockers(self):
        if self.phase != "fault":
            return ["not in a fault"]
        blockers = list(self.rig.faults())
        if self.startup_complete:
            blockers += [f"no {name} feedback" for name in self.missing_feedback()]
        return blockers

    def recover(self, now):
        blockers = self.recovery_blockers()
        if blockers:
            self.say(f"Cannot recover yet: {', '.join(blockers)}.", "warn")
            return
        self.release_required = True
        if not self.startup_complete:
            # Restart the startup sequence with the controllers already open.
            self.phase = "homing"
            self.home_sent = False
            self.can_start_homing = False
            self.settling.clear()
            self.done &= {"arms_open", "body_open", "body_cal"}
            self.arm_targets = {}
            self.deadline = now + HOME_TIMEOUT
            self.say("Restarting startup: homing both arms.")
            return
        errors = self.freeze(now, all_arms=True)
        if errors:
            self.say(f"Cannot recover yet: {', '.join(dict.fromkeys(errors))}.", "warn")
            return
        self.phase = "paused"
        self.fault_detail = None
        self.say("Recovered and holding the current pose. Press Enable to continue.")

    def resume(self, now):
        if self.missing_feedback():
            self.say(f"Cannot enable: no {', '.join(self.missing_feedback())} feedback.", "warn")
        elif self.phase == "paused":
            self.phase = self.resume_phase
            self.say("Back in mapping mode. Hold Space to drive." if self.phase == "mapping" else
                     "Enabled. Hold Space to use the controls.")
        elif self.phase in ("parked", "mapping"):
            self.begin_rest(now)

    # Operator input --------------------------------------------------------------

    def input(self, deadman, keys, action, now, speed=0, selection="both", drive_mode="latched"):
        self.speed_level = speed
        self.selection = selection
        self.drive_mode = drive_mode
        if self.phase not in DRIVE_PHASES:
            return
        if not deadman:
            self.release_required = False
            if self.armed or self.velocity != ZERO:
                self.brake()
            if self.arm_deadline is not None or self.body_deadlines:
                errors = self.freeze(now)
                if errors:
                    self.fault(", ".join(errors).capitalize() + ".", now)
            return
        if self.release_required:
            return
        velocity = drive_velocity(keys, speed)
        if self.rig.wheel_speeds(now) is None:
            if velocity != ZERO:
                self.block_drive("Wheel feedback unavailable. Release Space before retrying.")
                return
            self.armed = False
        else:
            self.armed = drive_mode == "latched" or velocity != ZERO
        if not self.armed and self.command != ZERO:
            self.brake()
        self.velocity = velocity if self.armed else ZERO
        if self.phase == "mapping" and action in MAPPING_BLOCKED:
            self.say("Arm, gripper, and torso keys are off in mapping mode. Return to rest to use them.")
        elif action in ARM_KEYS:
            self.jog_arms(action, now)
        elif action in BODY_KEYS:
            self.jog_body(action, now)
        elif action in GRIPPER_KEYS:
            sides = ARM_SELECTIONS[self.selection]
            self.rig.grip(GRIPPER_KEYS[action], sides)
            state = "Open requested" if action == "g" else "Close requested"
            for side in sides:
                self.gripper[side] = state
            self.say(f"{self.selection_label()} gripper{'s' if len(sides) > 1 else ''}: {state.lower()}.")

    def selection_label(self):
        return {"both": "Both", "left": "Left", "right": "Right"}[self.selection]

    def jog_arms(self, action, now):
        if self.arm_deadline is not None:
            self.say("Wait for the arms to finish their current step.")
            return
        sides = ARM_SELECTIONS[self.selection]
        axis, direction = ARM_KEYS[action]
        targets = dict(self.arm_targets)
        for side in sides:
            # Same XYZ offset for each selected arm, from its own target.
            pose = list(self.arm_targets.get(side, self.arm_state[side].pose))
            pose[axis] += direction * ARM_STEP_METERS
            if math.dist(pose[:3], self.arm_rest[side][:3]) > ARM_RADIUS_METERS + 1e-9:
                self.say(f"{side.capitalize()} arm jog limit reached ({ARM_RADIUS_METERS * 1000:.0f} mm from rest).")
                return
            targets[side] = tuple(pose)
        self.arm_deadline = now + JOG_TIMEOUT
        self.arm_moving = set(sides)
        for side in sides:
            self.rig.move_arm(side, targets[side])
        self.arm_targets = targets
        label = "Both arms" if len(sides) > 1 else f"{sides[0].capitalize()} arm"
        self.say(f"{label} jogged {'XYZ'[axis]}{'+' if direction > 0 else '−'}.")

    def jog_body(self, action, now):
        axis, step = BODY_KEYS[action]
        if axis not in self.body_state:
            self.say("That axis has no fresh feedback.")
            return
        if axis in self.body_deadlines:
            self.say("Wait for the axis to finish its current step.")
            return
        actual = self.body_state[axis]
        maximum, travel = BODY_LIMITS[axis]
        if not -BODY_TOLERANCES[axis] <= actual <= maximum:
            self.say("The axis is outside its jog range.")
            return
        if axis == 3:
            self.body_rest.setdefault(axis, max(0.0, actual))
        goal = round(self.body_targets.get(axis, max(0.0, actual)) + step, 6)
        if axis in (0, 1) and step < 0:
            goal = max(0.0, goal)
        if not 0.0 <= goal <= maximum or abs(goal - self.body_rest[axis]) > travel + 1e-9:
            self.say("Body jog limit reached.")
            return
        self.body_targets[axis] = goal
        self.body_deadlines[axis] = now + JOG_TIMEOUT
        name = {0: "Lift", 1: "Torso pivot", 3: "Head pitch"}[axis]
        self.say(f"{name} target {goal:.2f}.")

    # Status -----------------------------------------------------------------------

    def startup_status(self):
        steps = []
        first_open = True
        for key, label in STARTUP_STEPS:
            if key in self.done:
                state = "done"
            elif first_open and self.phase in ("homing", "resting"):
                state = "active"
            elif first_open and self.phase == "fault" and not self.startup_complete:
                state = "failed"
            else:
                state = "pending"
            if key not in self.done:
                first_open = False
            steps.append([label, state])
        return steps

    def status(self, now):
        if self.phase == "homing":
            arms = {side: ("Homed" if f"{side}_home" in self.done else
                           "Homing" if self.home_sent else
                           "Waiting for encoders" if side in self.opened else "Waiting")
                    for side in SIDES}
        else:
            arms = {side: ("Feedback lost" if not self.arm_state.get(side) else
                           {"ready": "Ready", "resting": "Moving to rest", "paused": "Paused",
                            "parking": "Going home", "parked": "Home", "fault": "Holding",
                            "stowing": "Folding", "mapping": "Folded"}[self.phase])
                    for side in SIDES}
        offsets = {}
        for side, rest in self.arm_rest.items():
            state = self.arm_state.get(side)
            if state is not None and self.phase in ("ready", "paused", "resting"):
                offsets[side] = [round(1000 * (value - origin), 2) for value, origin in zip(state.pose[:3], rest[:3])]
        ranges = {}
        for axis, rest in self.body_rest.items():
            maximum, travel = BODY_LIMITS[axis]
            ranges[AXIS_NAMES[axis]] = [max(0.0, rest - travel), min(maximum, rest + travel)]
        return {
            "phase": self.phase, "notice": self.notice, "arms": arms,
            "poses": {side: state.pose[:3] for side, state in self.arm_state.items() if state},
            "offsets": offsets, "arm_limit_mm": ARM_RADIUS_METERS * 1000,
            "body": {AXIS_NAMES[axis]: value for axis, value in self.body_state.items()},
            "body_targets": {AXIS_NAMES[axis]: value for axis, value in self.body_targets.items()},
            "body_ranges": ranges,
            "body_moving": [AXIS_NAMES[axis] for axis in sorted(self.body_deadlines)],
            "arms_moving": sorted(self.arm_moving),
            "velocity": self.velocity, "command": self.command, "armed": self.armed,
            "release_required": self.release_required,
            "wheels": self.rig.wheel_speeds(now), "wheel_warning": self.wheel_warning,
            "speed_level": self.speed_level, "speed_levels": SPEED_LEVELS, "yaw_per_linear": YAW_PER_LINEAR,
            "selection": self.selection, "drive_mode": self.drive_mode, "resume_phase": self.resume_phase,
            "gripper": self.gripper, "started": bool(self.opened),
            "startup": self.startup_status(), "startup_complete": self.startup_complete,
            "time_left": round(max(0.0, self.deadline - now), 1) if self.phase in TIMED_PHASES else None,
            "recoverable": self.phase == "fault" and not self.recovery_blockers(),
            "events": list(self.events)[-8:], "event_seq": self.event_seq,
        }

    def close(self):
        try:
            if self.opened:
                self.stop("Disconnected.", time.monotonic())
                self.brake()
                deadline = time.monotonic() + BRAKE_SECONDS
                while time.monotonic() < deadline:
                    self.rig.body_tick(self.body_targets, ZERO, False, calibrating=False)
                    time.sleep(0.01)
        finally:
            self.rig.close()


if __name__ == "__main__":
    arm_worker(sys.argv[1], int(sys.argv[2]))
