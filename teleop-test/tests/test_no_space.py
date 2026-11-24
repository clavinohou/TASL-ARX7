"""Input rules without the hold-Space requirement (the default)."""

from pathlib import Path
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mac_controller as mc  # noqa: E402
from tests.test_mac_controller import ready_controller, status  # noqa: E402


def controller():
    return ready_controller(require_space=False)


class NoSpace(unittest.TestCase):
    def test_default_does_not_require_space(self):
        self.assertFalse(mc.Controller(simulation=True).require_space)

    def test_drive_key_alone_drives(self):
        c = controller()
        c.update_input("tab", [], True)
        c.update_input("tab", ["w"], True)
        frame = status(c)
        self.assertTrue(frame["deadman"])
        self.assertEqual((frame["keys"], frame["drive_mode"]), (["w"], "latched"))

    def test_idle_focused_page_brakes_instead_of_holding_drive_mode(self):
        c = controller()
        c.update_input("tab", [], True)
        frame = status(c)
        self.assertTrue(frame["deadman"])
        self.assertEqual((frame["keys"], frame["drive_mode"]), ([], "pulse"))

    def test_drive_mode_lingers_briefly_after_release_then_brakes(self):
        c = controller()
        c.update_input("tab", [], True)
        c.update_input("tab", ["w"], True)
        status(c)
        c.update_input("tab", [], True)
        self.assertEqual(status(c)["drive_mode"], "latched")  # ramp down smoothly
        c.last_drive -= mc.DRIVE_LINGER + 0.01
        self.assertEqual(status(c)["drive_mode"], "pulse")

    def test_pulse_setting_still_brakes_on_key_release(self):
        c = controller()
        c.command("tab", "drive_mode", "pulse")
        c.update_input("tab", ["w"], True)
        c.update_input("tab", [], True)
        self.assertEqual(status(c)["drive_mode"], "pulse")

    def test_jog_tap_is_sent(self):
        c = controller()
        c.update_input("tab", [], True)
        c.update_input("tab", ["i"], True)
        c.update_input("tab", [], True)
        frame = status(c)
        self.assertEqual((frame["deadman"], frame["action"]), (True, "i"))

    def test_key_held_through_phase_change_is_ignored_until_pressed_again(self):
        c = controller()
        c.state["phase"] = "paused"
        c.update_input("tab", ["w"], True)
        self.assertEqual(status(c, "ready")["keys"], [])
        c.update_input("tab", [], True)
        c.update_input("tab", ["w"], True)
        self.assertEqual(status(c)["keys"], ["w"])

    def test_release_required_clears_only_when_all_keys_are_released(self):
        c = controller()
        c.update_input("tab", [], True)
        c.update_input("tab", ["w"], True)
        # Holding a key: stay "held", which the robot ignores while it waits for a release.
        c.on_status({"phase": "ready", "lease": 9, "release_required": True, "events": [], "event_seq": 0})
        self.assertTrue(c.sent[-1]["deadman"])
        c.update_input("tab", [], True)
        c.on_status({"phase": "ready", "lease": 10, "release_required": True, "events": [], "event_seq": 0})
        self.assertFalse(c.sent[-1]["deadman"])  # the release the robot needs
        c.on_status({"phase": "ready", "lease": 11, "release_required": False, "events": [], "event_seq": 0})
        self.assertTrue(c.sent[-1]["deadman"])

    def test_session_lockout_clears_through_the_mac_rules(self):
        """Stop, hold W through Resume, release, press W again: only the new press drives."""
        from tests.test_session import Harness
        c = controller()
        h = Harness().ready()
        c.state = h.session.status(h.now) | {"lease": 1}

        def exchange(keys):
            c.update_input("tab", keys, True)
            c.on_status(h.session.status(h.now) | {"lease": c.state["lease"] + 1, "events": [], "event_seq": 0})
            frame = c.sent[-1]
            h.session.input(frame["deadman"], set(frame["keys"]), frame["action"], h.now, 0, "both",
                            frame["drive_mode"])
            for _ in range(10):  # tick without Harness.run, which sends its own input frames
                h.now += 0.005
                h.session.tick(h.now)

        h.session.stop("Stopped.", h.now)
        h.session.resume(h.now)
        c.state["phase"] = "ready"
        exchange(["w"])
        exchange(["w"])
        self.assertFalse(h.session.armed)  # W was held through the stop
        exchange([])
        self.assertFalse(h.session.release_required)
        exchange(["w"])
        self.assertTrue(h.session.armed)

    def test_focus_loss_still_stops(self):
        c = controller()
        c.update_input("tab", ["w"], True)
        c.update_input("tab", ["w"], False)
        self.assertIn({"op": "stop"}, c.sent)

    def test_stale_page_input_drops_control(self):
        c = controller()
        c.update_input("tab", ["w"], True)
        c.browser_seen -= mc.BROWSER_FRESH + 0.05
        self.assertFalse(status(c)["deadman"])

    def test_no_control_outside_drive_phases(self):
        c = controller()
        c.state["phase"] = "paused"
        c.update_input("tab", ["w"], True)
        self.assertFalse(status(c, "paused")["deadman"])

    def test_space_can_be_required_again(self):
        c = controller()
        self.assertTrue(c.command("tab", "require_space", True)["ok"])
        c.update_input("tab", ["w"], True)
        self.assertFalse(status(c)["deadman"])
        c.update_input("tab", [], True)
        c.update_input("tab", ["space"], True)
        c.update_input("tab", ["space", "w"], True)
        self.assertEqual(status(c)["keys"], ["w"])


class SessionWithoutSpace(unittest.TestCase):
    """The robot session sees an always-held deadman while focused; jogs must still finish."""

    def test_arm_jog_completes_with_no_key_held(self):
        from tests.test_session import Harness
        h = Harness().ready()
        h.action("i")
        self.assertTrue(h.session.arm_moving)
        end = h.now + 1.5
        while h.session.arm_moving and h.now < end:
            h.now += 0.005
            if round(h.now / 0.005) % 10 == 0:
                h.session.input(True, set(), "", h.now, 0, "both", "pulse")
            h.session.tick(h.now)
        self.assertFalse(h.session.arm_moving)
        self.assertEqual(h.session.phase, "ready")


if __name__ == "__main__":
    unittest.main()
