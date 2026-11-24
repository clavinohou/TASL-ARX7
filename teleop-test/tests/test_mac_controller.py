"""Mac controller input rules, page server checks, and an end-to-end simulator run."""

import http.client
import json
from pathlib import Path
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mac_controller as mc  # noqa: E402


class FakeProcess:
    pass


def ready_controller(require_space=True):
    """Ready controller. These rules tests use the optional hold-Space mode; tests/test_no_space.py covers the default."""
    controller = mc.Controller(simulation=True)
    controller.require_space = require_space
    controller.process = FakeProcess()
    controller.sent = []
    controller.send = lambda frame: controller.sent.append(frame) or True
    controller.command("tab", "claim")
    controller.state = {"phase": "ready", "lease": 1}
    controller.last_status = time.monotonic()
    return controller


def status(controller, phase="ready", lease=None):
    lease = lease or (controller.state.get("lease", 0) + 1)
    controller.on_status({"phase": phase, "lease": lease, "events": [], "event_seq": 0})
    return controller.sent[-1]


class InputRules(unittest.TestCase):
    def test_space_then_key_drives(self):
        c = ready_controller()
        c.update_input("tab", ["space"], True)
        c.update_input("tab", ["space", "w"], True)
        frame = status(c)
        self.assertTrue(frame["deadman"])
        self.assertEqual(frame["keys"], ["w"])

    def test_key_held_before_space_is_ignored(self):
        c = ready_controller()
        c.update_input("tab", ["w"], True)
        c.update_input("tab", ["w", "space"], True)
        frame = status(c)
        self.assertTrue(frame["deadman"])
        self.assertEqual(frame["keys"], [])

    def test_tap_between_status_frames_is_not_lost(self):
        c = ready_controller()
        c.update_input("tab", ["space"], True)
        c.update_input("tab", ["space", "i"], True)
        c.update_input("tab", ["space"], True)
        self.assertEqual(status(c)["action"], "i")
        self.assertEqual(status(c)["action"], "")

    def test_jog_is_dropped_if_space_released_before_sending(self):
        c = ready_controller()
        c.update_input("tab", ["space"], True)
        c.update_input("tab", ["space", "i"], True)
        c.update_input("tab", [], True)
        frame = status(c)
        self.assertFalse(frame["deadman"])
        self.assertEqual(frame["action"], "")

    def test_space_held_through_phase_change_is_blocked(self):
        c = ready_controller()
        c.state["phase"] = "paused"
        c.update_input("tab", ["space"], True)
        frame = status(c, "ready")
        self.assertFalse(frame["deadman"])
        c.update_input("tab", [], True)
        c.update_input("tab", ["space"], True)
        self.assertTrue(status(c)["deadman"])

    def test_focus_loss_in_ready_stops(self):
        c = ready_controller()
        c.update_input("tab", ["space", "w"], True)
        c.update_input("tab", ["space", "w"], False)
        self.assertIn({"op": "stop"}, c.sent)
        self.assertFalse(status(c)["deadman"])

    def test_focus_loss_during_startup_does_not_stop(self):
        c = ready_controller()
        c.state["phase"] = "homing"
        c.update_input("tab", [], True)
        c.update_input("tab", [], False)
        self.assertNotIn({"op": "stop"}, c.sent)

    def test_stale_browser_input_drops_deadman(self):
        c = ready_controller()
        c.update_input("tab", ["space", "w"], True)
        c.browser_seen -= mc.BROWSER_FRESH + 0.05
        self.assertFalse(status(c)["deadman"])

    def test_silent_page_counts_as_focus_loss(self):
        c = ready_controller()
        c.update_input("tab", ["space"], True)
        c.browser_seen -= mc.BROWSER_GONE + 0.05
        c.tick(time.monotonic())
        self.assertIn({"op": "stop"}, c.sent)

    def test_other_tab_cannot_send_input(self):
        c = ready_controller()
        self.assertFalse(c.update_input("other", ["space", "w"], True))
        self.assertFalse(c.command("other", "enable")["ok"])

    def test_operation_is_retried_until_phase_changes(self):
        c = ready_controller()
        c.state["phase"] = "paused"
        self.assertTrue(c.command("tab", "enable")["ok"])
        self.assertEqual(status(c, "paused")["action"], "enable")
        self.assertEqual(status(c, "paused")["action"], "enable")
        self.assertEqual(status(c, "ready")["action"], "")

    def test_operation_not_allowed_in_wrong_phase(self):
        c = ready_controller()
        self.assertFalse(c.command("tab", "recover")["ok"])
        self.assertFalse(c.command("tab", "enable")["ok"])

    def test_settings_are_sent_with_every_frame(self):
        c = ready_controller()
        c.command("tab", "speed", 2)
        c.command("tab", "arms", "right")
        c.command("tab", "drive_mode", "pulse")
        frame = status(c)
        self.assertEqual((frame["speed"], frame["arms"], frame["drive_mode"]), (2, "right", "pulse"))
        self.assertFalse(c.command("tab", "speed", 7)["ok"])

    def test_heartbeat_loss_requests_stop(self):
        c = ready_controller()
        c.last_status -= mc.HEARTBEAT_WAIT + 0.1
        c.tick(time.monotonic())
        self.assertIn({"op": "stop"}, c.sent)
        self.assertTrue(c.waiting)

    def test_heartbeat_allows_sdk_open_during_startup(self):
        c = ready_controller()
        c.state["phase"] = "homing"
        c.last_status -= mc.HEARTBEAT_WAIT + 0.5
        c.tick(time.monotonic())
        self.assertNotIn({"op": "stop"}, c.sent)

    def test_robot_events_are_logged_once(self):
        c = ready_controller()
        event = {"phase": "ready", "lease": 5, "events": [[1, "warn", "Hello"]], "event_seq": 1}
        c.on_status(dict(event))
        c.on_status(dict(event, lease=6))
        self.assertEqual(sum(entry["text"] == "Hello" for entry in c.log), 1)


class PageServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = mc.Controller(simulation=True)
        cls.token = "test-token"
        cls.server = mc.make_server(cls.controller, 0, cls.token)
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.controller.closing = True
        with cls.controller.lock:
            cls.controller.notify()
        cls.server.shutdown()

    def request(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=3)
        connection.request(method, path, body=body, headers=headers or {})
        response = connection.getresponse()
        data = response.read() if not path.startswith("/events") else response.fp.readline()
        connection.close()
        return response.status, data

    def test_page_requires_token(self):
        self.assertEqual(self.request("GET", "/")[0], 403)
        status, body = self.request("GET", f"/?t={self.token}")
        self.assertEqual(status, 200)
        self.assertIn(b"ARX X7s Controller", body)

    def test_rejects_foreign_host_header(self):
        status, _ = self.request("GET", f"/?t={self.token}", headers={"Host": "evil.example:80"})
        self.assertEqual(status, 403)

    def test_post_requires_token_header(self):
        body = json.dumps({"client": "x", "op": "claim"})
        self.assertEqual(self.request("POST", "/api/command", body, {"Content-Type": "application/json"})[0], 403)
        status, data = self.request("POST", "/api/command", body,
                                    {"Content-Type": "application/json", "X-ARX-Token": self.token})
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(data)["ok"])

    def test_event_stream_sends_snapshot(self):
        status, line = self.request("GET", f"/events?t={self.token}")
        self.assertEqual(status, 200)
        self.assertTrue(line.startswith(b"data: "))
        self.assertIn("app", json.loads(line[6:]))


class EndToEnd(unittest.TestCase):
    """Real control_bridge subprocess in simulation, driven like the page drives it."""

    def setUp(self):
        self.controller = mc.Controller(simulation=True)
        threading.Thread(target=self.controller.run, daemon=True).start()
        self.controller.command("tab", "claim")
        self.stop_input = threading.Event()
        self.keys = []
        threading.Thread(target=self.browser, daemon=True).start()

    def tearDown(self):
        self.stop_input.set()
        self.controller.shutdown()

    def browser(self):
        while not self.stop_input.is_set():
            self.controller.update_input("tab", list(self.keys), True)
            time.sleep(0.03)

    def wait(self, condition, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.controller.lock:
                if condition(self.controller):
                    return True
            time.sleep(0.02)
        return False

    def test_connect_enable_drive_park_disconnect(self):
        c = self.controller
        c.command("tab", "connect")
        self.assertTrue(self.wait(lambda c: c.state.get("phase") == "idle"))
        self.assertTrue(c.command("tab", "enable")["ok"])
        self.assertTrue(self.wait(lambda c: c.state.get("phase") == "ready"), c.state.get("notice"))
        self.keys = ["space"]
        time.sleep(0.15)
        self.keys = ["space", "w"]
        self.assertTrue(self.wait(lambda c: c.state.get("armed") and c.state["command"][0] > 0.14))
        self.keys = ["space"]
        self.assertTrue(self.wait(lambda c: c.state.get("armed") and c.state["command"][0] == 0))
        self.keys = []
        self.assertTrue(self.wait(lambda c: not c.state.get("armed")))
        self.assertTrue(c.command("tab", "park")["ok"])
        self.assertTrue(self.wait(lambda c: c.state.get("phase") == "parked", 25), c.state.get("notice"))
        c.command("tab", "disconnect")
        self.assertTrue(self.wait(lambda c: not c.process))
        texts = [entry["text"] for entry in c.log]
        self.assertTrue(any("Parked" in text for text in texts))


if __name__ == "__main__":
    unittest.main()
