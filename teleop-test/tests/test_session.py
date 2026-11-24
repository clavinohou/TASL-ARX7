"""Simulator tests for startup, chassis control, parking, and fault recovery."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from jog import ZERO  # noqa: E402
import robot_session as rs  # noqa: E402
from robot_session import RobotSession, Simulator, drive_velocity, ramp  # noqa: E402

STEP = 0.005


class Harness:
    """Drive a session the way the bridge does: heartbeats plus fast ticks."""

    def __init__(self, rig=None):
        self.rig = rig or Simulator()
        self.now = 100.0
        self.session = RobotSession(self.rig, self.now)

    def run(self, seconds, deadman=False, keys=(), drive_mode="latched", speed=0, selection="both"):
        end = self.now + seconds
        while self.now < end:
            self.now += STEP
            if round(self.now / STEP) % 10 == 0:  # 20 Hz input frames
                self.session.input(deadman, set(keys), "", self.now, speed, selection, drive_mode)
                if not deadman and self.session.phase == "homing":
                    self.session.advance_startup()
            self.session.tick(self.now)

    def until(self, phase, limit=40.0, **kwargs):
        end = self.now + limit
        while self.session.phase != phase and self.now < end:
            self.run(0.05, **kwargs)
        return self.session.phase == phase

    def action(self, key, selection="both"):
        self.session.input(True, set(), key, self.now, 0, selection, "latched")

    def ready(self):
        assert self.until("ready"), self.session.notice
        self.run(0.1)  # release Space once so release_required clears
        return self


class DriveMath(unittest.TestCase):
    def test_turn_and_diagonals_stay_above_the_observed_deadband(self):
        # 49 chassis counts moved the base; 16 did not.
        self.assertAlmostEqual(drive_velocity({"left"})[2], 0.30)
        self.assertGreaterEqual(int(drive_velocity({"left"})[2] * 165), 49)
        for level in range(len(rs.SPEED_LEVELS)):
            x, y, _ = drive_velocity({"w", "a"}, level)
            self.assertGreaterEqual(int(min(abs(x), abs(y)) * 330), 49)
            self.assertLessEqual(math.hypot(x, y), rs.MAX_TRANSLATION + 1e-9)

    def test_opposite_keys_cancel(self):
        self.assertEqual(drive_velocity({"w", "s"}), (0.0, 0.0, 0.0))

    def test_ramp_rates(self):
        self.assertAlmostEqual(ramp(ZERO, (0.15, 0, 0), 0.1)[0], 0.06)
        self.assertAlmostEqual(ramp((0.15, 0, 0), ZERO, 0.05)[0], 0.075)
        self.assertEqual(ramp((0.14, 0, 0), (0.15, 0, 0), 0.1)[0], 0.15)


class Startup(unittest.TestCase):
    def test_reaches_ready_with_checklist_complete(self):
        h = Harness().ready()
        status = h.session.status(h.now)
        self.assertTrue(all(state == "done" for _, state in status["startup"]))
        self.assertAlmostEqual(h.session.body_targets[0], rs.REST_LIFT)
        self.assertLess(max(abs(value) for value in status["offsets"]["left"]), 0.5)  # at rest
        self.assertAlmostEqual(h.session.arm_rest["left"][1] - Simulator.HOME["left"][1], rs.REST_SPREAD)
        self.assertAlmostEqual(h.session.arm_rest["right"][1] - Simulator.HOME["right"][1], -rs.REST_SPREAD)

    def test_stop_during_homing_faults_and_recover_restarts(self):
        h = Harness()
        h.run(0.3)
        h.session.stop("Stopped.", h.now)
        self.assertEqual(h.session.phase, "fault")
        self.assertTrue(h.session.status(h.now)["recoverable"])
        h.session.recover(h.now)
        self.assertEqual(h.session.phase, "homing")
        self.assertTrue(h.until("ready"))


class Chassis(unittest.TestCase):
    def test_latched_mode_keeps_drive_mode_until_space_release(self):
        h = Harness().ready()
        h.run(0.5, deadman=True, keys={"w"})
        self.assertTrue(h.session.armed)
        self.assertAlmostEqual(h.session.command[0], 0.15)
        self.assertEqual(h.rig.modes[-1][0], 1)
        h.run(0.3, deadman=True)  # key released, Space still held
        self.assertTrue(h.session.armed)
        self.assertEqual(h.session.command, ZERO)
        self.assertEqual(h.rig.modes[-1], (1, ZERO))
        h.run(0.06)  # Space released
        self.assertFalse(h.session.armed)
        self.assertEqual(h.rig.modes[-1], (2, ZERO))

    def test_release_ramps_down_in_drive_mode(self):
        h = Harness().ready()
        h.run(0.5, deadman=True, keys={"w"})
        h.run(0.06, deadman=True)
        self.assertGreater(h.session.command[0], 0.0)
        self.assertLess(h.session.command[0], 0.15)
        self.assertTrue(h.session.armed)

    def test_pulse_mode_brakes_on_key_release(self):
        h = Harness().ready()
        h.run(0.5, deadman=True, keys={"w"}, drive_mode="pulse")
        self.assertTrue(h.session.armed)
        h.run(0.06, deadman=True, drive_mode="pulse")
        self.assertFalse(h.session.armed)
        self.assertEqual(h.rig.modes[-1], (2, ZERO))

    def test_space_release_is_immediate_brake(self):
        h = Harness().ready()
        h.run(0.5, deadman=True, keys={"w"})
        h.session.input(False, set(), "", h.now)
        h.session.tick(h.now + STEP)
        self.assertEqual(h.rig.modes[-1], (2, ZERO))

    def test_unproven_wheel_feedback_only_warns(self):
        h = Harness(Simulator(stall=True)).ready()
        h.run(1.6, deadman=True, keys={"w"})
        self.assertEqual(h.session.wheel_warning, "No wheel response")
        self.assertTrue(h.session.armed)
        self.assertIn("wheel feedback is not working", h.session.notice)

    def test_stall_after_proven_feedback_brakes_and_requires_release(self):
        h = Harness().ready()
        h.run(0.8, deadman=True, keys={"w"})
        self.assertTrue(h.session.wheels_proven)
        h.run(0.3, deadman=True)  # stop between drives, Space still held
        h.rig.stall = True
        h.run(1.6, deadman=True, keys={"w"})
        self.assertFalse(h.session.armed)
        self.assertEqual(h.session.wheel_warning, "No wheel response")
        self.assertTrue(h.session.release_required)
        self.assertEqual(h.session.phase, "ready")

    def test_moving_wheels_do_not_trigger_stall(self):
        h = Harness().ready()
        h.run(2.0, deadman=True, keys={"w"})
        self.assertTrue(h.session.armed)
        self.assertIsNone(h.session.wheel_warning)

    def test_missing_wheel_feedback_blocks_drive(self):
        h = Harness().ready()
        h.rig.wheel_speeds = lambda now: None
        h.run(0.2, deadman=True, keys={"w"})
        self.assertFalse(h.session.armed)
        self.assertIn("Wheel feedback", h.session.notice)

    def test_cut_drive_keeps_session_ready(self):
        h = Harness().ready()
        h.run(0.3, deadman=True, keys={"w"})
        h.session.cut_drive()
        self.assertEqual(h.session.phase, "ready")
        self.assertFalse(h.session.armed)
        self.assertTrue(h.session.release_required)
        h.run(0.2, deadman=True, keys={"w"})  # must release Space first
        self.assertFalse(h.session.armed)


class Jogs(unittest.TestCase):
    def test_single_arm_jog_moves_only_that_arm(self):
        h = Harness().ready()
        left, right = h.session.arm_targets["left"], h.session.arm_targets["right"]
        h.action("i", selection="left")
        self.assertAlmostEqual(h.session.arm_targets["left"][2], left[2] + rs.ARM_STEP_METERS)
        self.assertEqual(h.session.arm_targets["right"], right)
        h.run(0.6, deadman=True, selection="left")
        self.assertIsNone(h.session.arm_deadline)
        self.assertEqual(h.session.phase, "ready")

    def test_arm_radius_limit(self):
        h = Harness().ready()
        for _ in range(8):
            h.action("u")
            h.run(0.6, deadman=True)
        self.assertIn("limit", h.session.notice)
        offset = h.session.status(h.now)["offsets"]["left"]
        self.assertLessEqual(math.hypot(*offset), 25.5)

    def test_lift_jog_and_limit(self):
        h = Harness().ready()
        h.action("r")
        self.assertAlmostEqual(h.session.body_targets[0], 0.20)
        h.run(1.0, deadman=True)
        self.assertNotIn("lift", h.session.status(h.now)["body_moving"])


class StopParkRecover(unittest.TestCase):
    def test_stop_pause_resume(self):
        h = Harness().ready()
        h.session.stop("Stopped.", h.now)
        self.assertEqual(h.session.phase, "paused")
        h.session.resume(h.now)
        self.assertEqual(h.session.phase, "ready")

    def test_park_then_enable_returns_to_rest(self):
        h = Harness().ready()
        h.action("r")
        h.run(1.0, deadman=True)
        h.session.begin_park(h.now)
        self.assertEqual(h.session.phase, "parking")
        self.assertTrue(h.until("parked"), h.session.notice)
        self.assertLess(h.rig.position[0], 0.04)
        self.assertTrue(all(h.session.arm_state[side].home() for side in rs.SIDES))
        h.session.resume(h.now)
        self.assertEqual(h.session.phase, "resting")
        self.assertTrue(h.until("ready"))

    def test_stop_while_parking_pauses(self):
        h = Harness().ready()
        h.session.begin_park(h.now)
        h.run(0.2)
        h.session.stop("Stopped.", h.now)
        self.assertEqual(h.session.phase, "paused")

    def test_feedback_loss_faults_then_recovers(self):
        h = Harness().ready()
        h.rig.withheld.add("left")
        h.run(0.1)
        self.assertEqual(h.session.phase, "fault")
        h.session.recover(h.now)
        self.assertEqual(h.session.phase, "fault")
        self.assertIn("no left arm feedback", h.session.notice)
        h.rig.withheld.clear()
        h.run(0.1)
        self.assertTrue(h.session.status(h.now)["recoverable"])
        h.session.recover(h.now)
        self.assertEqual(h.session.phase, "paused")
        h.session.resume(h.now)
        self.assertEqual(h.session.phase, "ready")

    def test_sdk_error_is_not_recoverable(self):
        h = Harness().ready()
        h.rig.errors.append("left arm: SDK process ended")
        h.run(0.1)
        self.assertEqual(h.session.phase, "fault")
        self.assertFalse(h.session.status(h.now)["recoverable"])

    def test_events_are_numbered(self):
        h = Harness().ready()
        events = h.session.status(h.now)["events"]
        self.assertTrue(events)
        self.assertEqual([seq for seq, _, _ in events], sorted(seq for seq, _, _ in events))


if __name__ == "__main__":
    unittest.main()
