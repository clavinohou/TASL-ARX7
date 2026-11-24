"""Whole-robot controller over a leased SSH standard input/output session.

Every status line offers a lease. The Mac answers with the lease and its input,
so input is accepted only while it is a fresh reply to a recent status.
"""

import argparse
import json
import os
import select
import signal
import sys
import time

from jog import TICK_SECONDS, ZERO
from robot_session import (
    ARM_SELECTIONS, DRIVE_KEYS, DRIVE_MODES, JOG_KEYS, OPERATIONS, SPEED_LEVELS,
    RobotHardware, RobotSession, Simulator, probe_sdk,
)

PREFIX = b"ARX3 "
OLD_PREFIXES = (b"ARX1 ", b"ARX2 ")
OFFER_INTERVAL = 0.05
OFFER_LIFETIME = 0.30  # oldest status a reply may answer
DRIVE_TIMEOUT = 0.25   # brake the chassis without fresh input
INPUT_TIMEOUT = 1.0    # pause the whole robot without fresh input
LINK_TIMEOUT = 60.0    # exit (and release the lock) if the Mac sends nothing at all
SDK_OPEN_LIMIT = 5.0
FRAME_LIMIT = 512


class Bridge:
    def __init__(self, live, stall=False):
        self.live = live
        self.stall = stall
        self.session = None
        self.lease = 0
        self.offers = {}
        self.last_input = None
        self.rtt = None
        self.notice = "Connected. Press Enable when the robot is clear." if live else "Connected. Press Enable to start."

    def stop(self, notice, now):
        self.offers.clear()
        self.last_input = None
        if self.session:
            self.session.stop(notice, now)

    def valid(self, frame):
        deadman = frame.get("deadman")
        keys = frame.get("keys", [])
        action = frame.get("action", "")
        speed = frame.get("speed", 0)
        return (type(deadman) is bool and isinstance(keys, list) and len(keys) <= len(DRIVE_KEYS)
                and all(isinstance(key, str) and key in DRIVE_KEYS for key in keys)
                and isinstance(action, str) and action in (JOG_KEYS | OPERATIONS | {""})
                and type(speed) is int and 0 <= speed < len(SPEED_LEVELS)
                and frame.get("arms", "both") in ARM_SELECTIONS
                and frame.get("drive_mode", DRIVE_MODES[0]) in DRIVE_MODES
                and not (action in OPERATIONS and (deadman or keys)))

    def accept(self, frame, now):
        if frame.get("op") == "stop":
            self.stop("Stopped. Press Enable to continue.", now)
            return
        lease = frame.get("lease")
        if type(lease) is not int:
            return
        issued = self.offers.pop(lease, None)
        if issued is None or now - issued >= OFFER_LIFETIME:
            return
        self.offers = {key: timestamp for key, timestamp in self.offers.items() if key > lease}
        if not self.valid(frame):
            self.stop("Invalid input. Motion stopped.", now)
            return
        self.last_input = now
        sample = now - issued
        self.rtt = sample if self.rtt is None else 0.8 * self.rtt + 0.2 * sample
        action = frame.get("action", "")
        if action == "enable":
            if self.session is None:
                try:
                    rig = RobotHardware() if self.live else Simulator(stall=self.stall)
                    self.session = RobotSession(rig, now)
                except (OSError, RuntimeError, ValueError) as exc:
                    self.notice = str(exc)
            else:
                self.session.resume(now)
            # The Enable request cannot itself open a motor controller.
            return
        if self.session is None:
            return
        try:
            if action == "park":
                self.session.begin_park(now)
                return
            if action == "recover":
                self.session.recover(now)
                return
            if action == "map":
                self.session.begin_map(now)
                return
            self.session.input(frame["deadman"], set(frame.get("keys", [])), action, now,
                               frame.get("speed", 0), frame.get("arms", "both"),
                               frame.get("drive_mode", DRIVE_MODES[0]))
            if not frame["deadman"] and self.session.phase == "homing":
                self.session.advance_startup()
                elapsed = time.monotonic() - now
                if elapsed > DRIVE_TIMEOUT:
                    self.offers.clear()
                    self.last_input = time.monotonic()
                if elapsed > SDK_OPEN_LIMIT:
                    self.session.fault("Opening the SDK took too long.", time.monotonic())
        except (OSError, RuntimeError, ValueError) as exc:
            self.session.fault(str(exc), time.monotonic())

    def tick(self, now):
        if self.session:
            if self.last_input is not None:
                quiet = now - self.last_input
                if quiet >= INPUT_TIMEOUT:
                    self.stop("Input timed out. Press Enable to continue.", now)
                elif quiet >= DRIVE_TIMEOUT:
                    self.session.cut_drive()
            self.session.tick(now)

    def offer(self, now):
        self.lease += 1
        self.offers = {key: sent for key, sent in self.offers.items() if now - sent < OFFER_LIFETIME}
        self.offers[self.lease] = now
        status = self.session.status(now) if self.session else {
            "phase": "idle", "notice": self.notice, "arms": {}, "poses": {}, "body": {},
            "wheels": None, "velocity": ZERO, "command": ZERO, "armed": False, "started": False,
            "speed_levels": SPEED_LEVELS, "events": [], "event_seq": 0,
        }
        status.update(lease=self.lease, live=self.live,
                      rtt_ms=None if self.rtt is None else round(1000 * self.rtt, 1))
        try:
            os.write(sys.stdout.fileno(), PREFIX + json.dumps(status, allow_nan=False).encode() + b"\n")
        except BlockingIOError:
            pass

    def close(self):
        if self.session:
            self.session.close()


def run(bridge):
    os.set_blocking(sys.stdout.fileno(), False)
    pending = b""
    next_offer = 0.0
    last_bytes = time.monotonic()
    while True:
        now = time.monotonic()
        bridge.tick(now)
        if now >= next_offer:
            bridge.offer(now)
            next_offer = now + OFFER_INTERVAL
        if now - last_bytes >= LINK_TIMEOUT:
            print("Controller stopped: no input from the Mac for "
                  f"{LINK_TIMEOUT:.0f} s.", file=sys.stderr, flush=True)
            return
        readable, _, _ = select.select([sys.stdin], [], [], TICK_SECONDS)
        if not readable:
            continue
        data = os.read(sys.stdin.fileno(), 4096)
        if not data:
            return
        last_bytes = time.monotonic()
        lines = (pending + data).split(b"\n")
        pending = lines.pop()
        if len(pending) > FRAME_LIMIT or any(len(line) > FRAME_LIMIT for line in lines):
            raise ValueError("Control message exceeds the allowed size")
        frames = [json.loads(line) for line in lines]
        if not all(isinstance(frame, dict) for frame in frames):
            raise ValueError("Expected a control message object")
        if any(frame.get("op") == "quit" for frame in frames):
            return
        if any(frame.get("op") == "stop" for frame in frames):
            bridge.stop("Stopped. Press Enable to continue.", time.monotonic())
            continue
        for frame in frames:
            bridge.accept(frame, time.monotonic())


def interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", action="store_true", help="Load the ARX SDK and permit explicit activation")
    parser.add_argument("--sim-stall", action="store_true", help="Simulator: wheels never report motion")
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)
    bridge = Bridge(args.robot, stall=args.sim_stall)
    try:
        if args.robot:
            probe_sdk()
        run(bridge)
    except (BrokenPipeError, KeyboardInterrupt):
        return 0
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f"Controller stopped: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        bridge.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
