"""The lift ceiling keeps clearance below the ZED Mini mounted above the lift."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import jog  # noqa: E402
from jog import BODY_JOG, LIFT_CEILING, LIFT_SDK_MAX, ZERO, Hardware  # noqa: E402
import robot_session as rs  # noqa: E402
from tests.test_session import Harness  # noqa: E402


class FakeBody:
    def __init__(self):
        self.heights = []

    def set_height(self, height):
        self.heights.append(height)

    def set_waist_pos(self, pos):
        pass

    def set_head_pitch(self, pitch):
        pass

    def set_chassis_cmd(self, x, y, yaw, mode):
        pass

    def loop(self):
        pass


class LiftCeiling(unittest.TestCase):
    def test_ceiling_is_below_sdk_travel(self):
        self.assertLess(LIFT_CEILING, LIFT_SDK_MAX)
        self.assertEqual(BODY_JOG["lift"][4], LIFT_CEILING)
        self.assertEqual(rs.BODY_LIMITS[0][0], LIFT_CEILING)
        self.assertLess(rs.ZERO_CHECK[0], LIFT_CEILING)

    def test_sdk_never_receives_a_height_above_the_ceiling(self):
        hardware = object.__new__(Hardware)
        hardware.body = FakeBody()
        for height in (LIFT_SDK_MAX, LIFT_CEILING + 0.01, 22.0):
            hardware.body_tick((height, 0.0, 0.0, 0.0), ZERO, 2)
            self.assertEqual(hardware.body.heights[-1], LIFT_CEILING)
        hardware.body_tick((3.0, 0.0, 0.0, 0.0), ZERO, 2)
        self.assertEqual(hardware.body.heights[-1], 3.0)

    def test_controller_refuses_a_lift_jog_past_the_ceiling(self):
        h = Harness().ready()
        near = LIFT_CEILING - 0.05
        h.session.body_rest[0] = near
        h.session.body_targets[0] = near
        h.run(1.0)
        h.action("r")
        self.assertIn("limit", h.session.notice)
        self.assertLessEqual(h.session.body_targets[0], LIFT_CEILING)
        self.assertLessEqual(h.session.status(h.now)["body_ranges"]["lift"][1], LIFT_CEILING)

    def test_startup_accepts_the_lift_reading_seen_on_its_bottom_stop(self):
        # On the robot the lift read 0.20 SDK units right after calibration.
        rig = rs.Simulator()
        rig.position[0] = 0.20
        h = Harness(rig)
        self.assertTrue(h.until("ready"), h.session.notice)

    def test_startup_refuses_a_lift_reading_far_from_zero(self):
        rig = rs.Simulator()
        rig.position[0] = rs.ZERO_CHECK[0] + 0.1
        h = Harness(rig)
        self.assertTrue(h.until("fault"), h.session.notice)
        self.assertIn("calibrated zero", h.session.notice)

    def test_jog_console_lift_range_stops_at_the_ceiling(self):
        _, step, _, minimum, maximum = jog.BODY_JOG["lift"]
        self.assertGreater(step, 0)
        self.assertEqual((minimum, maximum), (0.0, LIFT_CEILING))


if __name__ == "__main__":
    unittest.main()
