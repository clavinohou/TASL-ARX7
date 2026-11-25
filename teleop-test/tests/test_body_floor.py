"""The lift and torso read slightly above zero on their stops after calibration (robot, 2026-11-24)."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import robot_session as rs  # noqa: E402
from tests.test_session import Harness  # noqa: E402

LIFT_STOP, TORSO_STOP = 0.20, 0.066  # readings seen on the robot after SDK calibration


def robot_like():
    rig = rs.Simulator()
    rig.stops = {0: LIFT_STOP, 1: TORSO_STOP}
    rig.position.update({0: LIFT_STOP, 1: TORSO_STOP})
    return Harness(rig)


class BodyFloor(unittest.TestCase):
    def test_simulator_stops_reproduce_the_old_fault(self):
        # Before the floor fix, rest asked for lift 0.10 and torso 0.0: unreachable.
        h = robot_like()
        h.rig.body_targets = {0: 0.10, 1: 0.0}
        h.rig.observe(0.0)
        self.assertEqual((h.rig.position[0], h.rig.position[1]), (LIFT_STOP, TORSO_STOP))

    def test_startup_reaches_ready_with_rest_at_the_floor(self):
        h = robot_like()
        self.assertTrue(h.until("ready"), h.session.notice)
        self.assertAlmostEqual(h.session.body_floor[0], LIFT_STOP, places=2)
        self.assertAlmostEqual(h.session.body_targets[0], h.session.body_floor[0])
        self.assertAlmostEqual(h.session.body_targets[1], h.session.body_floor[1])

    def test_park_and_mapping_settle_on_the_floor(self):
        h = robot_like()
        self.assertTrue(h.until("ready"), h.session.notice)
        h.run(0.1)
        h.session.begin_map(h.now)
        self.assertTrue(h.until("mapping"), h.session.notice)
        self.assertAlmostEqual(h.session.body_targets[1], h.session.body_floor[1])
        h.session.begin_park(h.now)
        self.assertTrue(h.until("parked"), h.session.notice)
        self.assertAlmostEqual(h.rig.position[0], h.session.body_floor[0], places=2)

    def test_lift_cannot_be_jogged_below_the_floor(self):
        h = robot_like()
        self.assertTrue(h.until("ready"), h.session.notice)
        h.run(0.1)
        h.action("f")
        h.run(1.0, deadman=True)
        self.assertGreaterEqual(h.session.body_targets[0], h.session.body_floor[0] - 1e-9)
        h.action("f")  # a second step down clamps to the floor rather than going below it
        h.run(1.0, deadman=True)
        self.assertAlmostEqual(h.session.body_targets[0], h.session.body_floor[0])
        low = h.session.status(h.now)["body_ranges"]["lift"][0]
        self.assertGreaterEqual(low, h.session.body_floor[0] - 1e-9)


if __name__ == "__main__":
    unittest.main()
