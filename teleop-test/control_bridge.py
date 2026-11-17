"""Whole-robot controller over a leased SSH standard input/output session."""

import argparse
import json
import os
import select
import signal
import sys
import time

from jog import TICK_SECONDS, ZERO
from robot_session import DRIVE_KEYS, JOG_KEYS, RobotHardware, RobotSession, Simulator, probe_sdk

PREFIX = b"ARX2 "
OFFER_INTERVAL = 0.05
OFFER_LIFETIME = 0.18
INPUT_TIMEOUT = 0.25
SDK_OPEN_LIMIT = 5.0
FRAME_LIMIT = 512


class Bridge:
    def __init__(self, live):
        self.live = live
        self.session = None
        self.lease = 0
        self.offers = {}
        self.last_input = None
        self.notice = "Connected. Press Enable when the robot is clear." if live else "Connected. Press Enable to start."

    def stop(self, notice, now):
        self.offers.clear()
        self.last_input = None
        if self.session:
            self.session.stop(notice, now)

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
        deadman = frame.get("deadman")
        keys = frame.get("keys", [])
        action = frame.get("action", "")
        if (type(deadman) is not bool or not isinstance(keys, list) or len(keys) > len(DRIVE_KEYS)
                or any(not isinstance(key, str) or key not in DRIVE_KEYS for key in keys)
                or not isinstance(action, str) or action not in (JOG_KEYS | {"", "enable"})):
            self.stop("Invalid input. Motion stopped.", now)
            return
        self.last_input = now
        if action == "enable" and not deadman and not keys:
            if self.session is None:
                try:
                    rig = RobotHardware() if self.live else Simulator()
                    self.session = RobotSession(rig, now)
                except (OSError, RuntimeError, ValueError) as exc:
                    self.notice = str(exc)
            elif self.session.phase == "paused":
                self.session.resume()
            # The Enable request cannot itself open a motor controller.
            return
        if self.session is None:
            return
        try:
            self.session.input(deadman, set(keys), action, now)
            if not deadman and self.session.phase == "homing":
                self.session.advance_startup()
                elapsed = time.monotonic() - now
                if elapsed > INPUT_TIMEOUT:
                    self.offers.clear()
                    self.last_input = time.monotonic()
                if elapsed > SDK_OPEN_LIMIT:
                    self.session.fault("Opening the SDK took too long.", time.monotonic())
        except (OSError, RuntimeError, ValueError) as exc:
            self.session.fault(str(exc), time.monotonic())

    def tick(self, now):
        if self.session:
            if self.last_input is not None and now - self.last_input >= INPUT_TIMEOUT:
                self.stop("Input timed out. Press Enable to continue.", now)
            self.session.tick(now)

    def offer(self, now):
        self.lease += 1
        self.offers = {key: sent for key, sent in self.offers.items() if now - sent < OFFER_LIFETIME}
        self.offers[self.lease] = now
        status = self.session.status(now) if self.session else {
            "phase": "idle", "notice": self.notice, "arms": {}, "poses": {}, "body": {},
            "wheels": None, "velocity": ZERO, "gripper": "—", "started": False,
        }
        status.update(lease=self.lease, live=self.live)
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
    while True:
        now = time.monotonic()
        bridge.tick(now)
        if now >= next_offer:
            bridge.offer(now)
            next_offer = now + OFFER_INTERVAL
        readable, _, _ = select.select([sys.stdin], [], [], TICK_SECONDS)
        if not readable:
            continue
        data = os.read(sys.stdin.fileno(), 4096)
        if not data:
            return
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
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)
    bridge = Bridge(args.robot)
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
