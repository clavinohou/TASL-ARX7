"""Lease, timeout, and validation tests for the control bridge."""

from pathlib import Path
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import control_bridge as cb  # noqa: E402
from control_bridge import Bridge  # noqa: E402

STEP = 0.005


class FakeClock:
    def __init__(self, bridge):
        self.bridge = bridge
        self.now = time.monotonic()  # the bridge times SDK opening on the real clock
        self.lease = None

    def offer(self):
        # Capture the lease number without writing to stdout.
        self.bridge.lease += 1
        self.bridge.offers[self.bridge.lease] = self.now
        self.lease = self.bridge.lease

    def reply(self, delay=0.01, **frame):
        frame.setdefault("deadman", False)
        frame.setdefault("keys", [])
        frame.setdefault("action", "")
        frame["lease"] = self.lease
        self.bridge.accept(frame, self.now + delay)

    def run(self, seconds, **frame):
        end = self.now + seconds
        while self.now < end:
            self.now += STEP
            if round(self.now / STEP) % 10 == 0:
                self.offer()
                self.reply(**frame)
            self.bridge.tick(self.now)

    def ready(self):
        self.offer()
        self.reply(action="enable")
        for _ in range(1000):
            self.run(0.05)
            if self.bridge.session.phase == "ready":
                return
        raise AssertionError(self.bridge.session.notice)


class Leases(unittest.TestCase):
    def setUp(self):
        self.bridge = Bridge(live=False)
        self.clock = FakeClock(self.bridge)

    def test_enable_creates_simulated_session(self):
        self.clock.ready()
        self.assertEqual(self.bridge.session.phase, "ready")
        self.assertIsNotNone(self.bridge.rtt)

    def test_stale_reply_is_ignored(self):
        self.clock.offer()
        self.clock.reply(delay=cb.OFFER_LIFETIME + 0.01, action="enable")
        self.assertIsNone(self.bridge.session)

    def test_reused_lease_is_ignored(self):
        self.clock.offer()
        self.clock.reply(action="enable")
        session = self.bridge.session
        self.clock.reply(action="enable")
        self.assertIs(self.bridge.session, session)

    def test_invalid_frame_stops(self):
        self.clock.ready()
        self.clock.run(0.1)
        self.clock.offer()
        self.clock.reply(deadman=True, keys=["w"], speed=9)
        self.assertEqual(self.bridge.session.phase, "paused")

    def test_operation_with_deadman_is_invalid(self):
        self.clock.ready()
        self.clock.offer()
        self.clock.reply(deadman=True, action="park")
        self.assertEqual(self.bridge.session.phase, "paused")


class Timeouts(unittest.TestCase):
    def setUp(self):
        self.bridge = Bridge(live=False)
        self.clock = FakeClock(self.bridge)
        self.clock.ready()
        self.clock.run(0.1)

    def silent(self, seconds):
        end = self.clock.now + seconds
        while self.clock.now < end:
            self.clock.now += STEP
            self.bridge.tick(self.clock.now)

    def test_short_gap_brakes_chassis_but_stays_ready(self):
        self.clock.run(0.4, deadman=True, keys=["w"])
        self.assertTrue(self.bridge.session.armed)
        self.silent(0.3)
        self.assertFalse(self.bridge.session.armed)
        self.assertEqual(self.bridge.session.phase, "ready")

    def test_long_gap_pauses(self):
        self.silent(cb.INPUT_TIMEOUT + 0.05)
        self.assertEqual(self.bridge.session.phase, "paused")
        self.assertIn("timed out", self.bridge.session.notice)


class Operations(unittest.TestCase):
    def setUp(self):
        self.bridge = Bridge(live=False)
        self.clock = FakeClock(self.bridge)
        self.clock.ready()
        self.clock.run(0.1)

    def test_park_and_enable(self):
        self.clock.offer()
        self.clock.reply(action="park")
        self.assertEqual(self.bridge.session.phase, "parking")
        for _ in range(400):
            self.clock.run(0.05)
            if self.bridge.session.phase == "parked":
                break
        self.assertEqual(self.bridge.session.phase, "parked")
        self.clock.offer()
        self.clock.reply(action="enable")
        self.assertEqual(self.bridge.session.phase, "resting")

    def test_recover(self):
        self.bridge.session.rig.withheld.add("lift")
        self.clock.run(0.1)
        self.assertEqual(self.bridge.session.phase, "fault")
        self.bridge.session.rig.withheld.clear()
        self.clock.run(0.1)
        self.clock.offer()
        self.clock.reply(action="recover")
        self.assertEqual(self.bridge.session.phase, "paused")

    def test_speed_and_selection_are_forwarded(self):
        self.clock.run(0.2, deadman=True, keys=["w"], speed=2, arms="left")
        self.assertEqual(self.bridge.session.speed_level, 2)
        self.assertEqual(self.bridge.session.selection, "left")
        self.assertAlmostEqual(self.bridge.session.velocity[0], 1.20)


if __name__ == "__main__":
    unittest.main()
