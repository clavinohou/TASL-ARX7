"""Mapping mode (session, bridge, Mac app) and the camera and live map plumbing."""

import http.client
import io
from pathlib import Path
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import camera_feed as cf  # noqa: E402
from control_bridge import Bridge  # noqa: E402
import mac_controller as mc  # noqa: E402
import zed_stream as zs  # noqa: E402
from tests.test_bridge import FakeClock  # noqa: E402
from tests.test_mac_controller import ready_controller, status  # noqa: E402
from tests.test_session import Harness  # noqa: E402

try:
    import numpy
except ImportError:  # the Mac's system Python has no NumPy; the robot does
    numpy = None


def mapping_harness():
    h = Harness().ready()
    h.session.begin_map(h.now)
    assert h.session.phase == "stowing"
    assert h.until("mapping"), h.session.notice
    h.run(0.1)  # release Space once after the phase change
    return h


class SessionMapping(unittest.TestCase):
    def test_stow_folds_arms_home_keeps_lift_and_holds_torso_upright(self):
        h = Harness().ready()
        lift = h.session.body_targets[0]
        h.session.begin_map(h.now)
        self.assertTrue(h.until("mapping"), h.session.notice)
        self.assertTrue(all(h.session.arm_state[side].home() for side in ("left", "right")))
        self.assertEqual(h.session.body_targets[0], lift)
        self.assertEqual(h.session.body_targets[1], 0.0)
        self.assertEqual(h.session.status(h.now)["arms"]["left"], "Folded")

    def test_drive_is_allowed_in_mapping(self):
        h = mapping_harness()
        h.run(0.3, deadman=True, keys={"w"})
        self.assertTrue(h.session.armed)
        self.assertGreater(h.session.command[0], 0.0)
        h.run(0.1)
        self.assertFalse(h.session.armed)

    def test_arm_gripper_and_torso_keys_are_off_in_mapping(self):
        h = mapping_harness()
        for key in ("i", "g", "q"):
            h.action(key)
            self.assertIn("off in mapping mode", h.session.notice)
        self.assertEqual(h.session.arm_targets, {})
        self.assertFalse(h.session.body_deadlines)
        self.assertEqual(h.rig.grips, {})

    def test_lift_and_head_jogs_still_work_in_mapping(self):
        h = mapping_harness()
        h.action("1")
        self.assertIn(3, h.session.body_deadlines)

    def test_stop_in_mapping_resumes_to_mapping(self):
        h = mapping_harness()
        h.session.stop("Stopped.", h.now)
        self.assertEqual(h.session.phase, "paused")
        self.assertEqual(h.session.status(h.now)["resume_phase"], "mapping")
        h.session.resume(h.now)
        self.assertEqual(h.session.phase, "mapping")

    def test_stop_while_folding_pauses_and_resumes_to_ready(self):
        h = Harness().ready()
        h.session.begin_map(h.now)
        h.run(0.05)
        h.session.stop("Stopped.", h.now)
        self.assertEqual(h.session.phase, "paused")
        h.session.resume(h.now)
        self.assertEqual(h.session.phase, "ready")

    def test_enable_from_mapping_returns_to_rest(self):
        h = mapping_harness()
        h.session.resume(h.now)
        self.assertEqual(h.session.phase, "resting")
        self.assertTrue(h.until("ready"), h.session.notice)

    def test_park_from_mapping(self):
        h = mapping_harness()
        h.session.begin_park(h.now)
        self.assertEqual(h.session.phase, "parking")
        self.assertTrue(h.until("parked"), h.session.notice)

    def test_fold_timeout_faults(self):
        h = Harness().ready()
        h.rig.withheld.add("left")
        h.session.begin_map(h.now)
        h.run(0.2)
        self.assertEqual(h.session.phase, "fault")


class BridgeMapping(unittest.TestCase):
    def test_map_operation_starts_folding(self):
        bridge = Bridge(live=False)
        clock = FakeClock(bridge)
        clock.ready()
        clock.run(0.1)
        clock.offer()
        clock.reply(action="map")
        self.assertEqual(bridge.session.phase, "stowing")

    def test_map_with_deadman_is_invalid(self):
        bridge = Bridge(live=False)
        clock = FakeClock(bridge)
        clock.ready()
        clock.offer()
        clock.reply(deadman=True, action="map")
        self.assertEqual(bridge.session.phase, "paused")


class MacMapping(unittest.TestCase):
    def test_deadman_drives_in_mapping(self):
        c = ready_controller()
        c.state["phase"] = "mapping"
        c.update_input("tab", ["space"], True)
        c.update_input("tab", ["space", "w"], True)
        frame = status(c, "mapping")
        self.assertTrue(frame["deadman"])
        self.assertEqual(frame["keys"], ["w"])

    def test_focus_loss_in_mapping_stops(self):
        c = ready_controller()
        c.state["phase"] = "mapping"
        c.update_input("tab", ["space"], True)
        c.update_input("tab", ["space"], False)
        self.assertIn({"op": "stop"}, c.sent)

    def test_map_operation_phase_rules(self):
        c = ready_controller()
        self.assertTrue(c.command("tab", "map")["ok"])
        self.assertEqual(status(c)["action"], "map")
        c.op = None
        c.state["phase"] = "mapping"
        self.assertFalse(c.command("tab", "map")["ok"])
        self.assertTrue(c.command("tab", "enable")["ok"])

    def test_auto_map_builds_only_with_arms_folded(self):
        c = ready_controller()
        sent = []

        class Camera:
            def send(self, message):
                sent.append(message)

        c.camera = Camera()
        c.update_integration()
        self.assertEqual(sent[-1], {"op": "integrate", "on": False})
        c.state = {"phase": "mapping", "lease": 2}
        c.update_integration()
        self.assertEqual(sent[-1], {"op": "integrate", "on": True})
        c.state = {"phase": "paused", "resume_phase": "mapping", "lease": 3}
        c.update_integration()
        self.assertEqual(len(sent), 2)  # still building while paused with the arms folded
        c.state = {"phase": "resting", "lease": 4}
        c.update_integration()
        self.assertEqual(sent[-1], {"op": "integrate", "on": False})
        c.command("tab", "map_mode", "on")
        self.assertEqual(sent[-1], {"op": "integrate", "on": True})
        self.assertFalse(c.command("tab", "map_mode", "sometimes")["ok"])


class Protocol(unittest.TestCase):
    def test_messages_round_trip(self):
        records = zs.VOXEL.pack(1, -2, 3, 10, 20, 30) + zs.VOXEL.pack(-4, 5, -6, 40, 50, 60)
        stream = io.BytesIO(zs.pack("color", b"jpeg") + zs.pack("voxels", zs.EPOCH.pack(7) + records))
        messages = list(zs.read_messages(stream))
        self.assertEqual(messages[0], ("color", b"jpeg"))
        self.assertEqual(zs.unpack_voxels(messages[1][1]), (7, records))

    def test_out_of_sync_stream_is_rejected(self):
        with self.assertRaises(ValueError):
            list(zs.read_messages(io.BytesIO(b"garbage-bytes-here")))

    @unittest.skipIf(numpy is None, "NumPy is not installed")
    def test_voxel_map_confirms_after_two_frames_and_filters_range(self):
        np = numpy
        vmap = zs.VoxelMap(np)
        xyz = np.array([[1.01, 0.01, 0.01], [1.02, 0.02, 0.02], [0.1, 0, 0], [np.nan, 0, 0]], dtype=np.float32)
        rgb = np.array([[200, 0, 0], [100, 0, 0], [0, 255, 0], [0, 0, 0]], dtype=np.uint8)
        self.assertEqual(vmap.add(xyz, rgb, (0, 0, 0), 0.45, 4.0), b"")
        records = vmap.add(xyz, rgb, (0, 0, 0), 0.45, 4.0)
        self.assertEqual(len(records), zs.VOXEL.size)
        self.assertEqual(zs.VOXEL.unpack(records), (20, 0, 0, 150, 0, 0))


class Hub(unittest.TestCase):
    def test_map_log_pages_and_resets_on_new_epoch(self):
        hub = cf.CameraHub()
        record = zs.VOXEL.pack(1, 2, 3, 4, 5, 6)
        hub.put_voxels(1, record * 3)
        epoch, start, end, data = hub.wait_map(0, 5, timeout=0)
        self.assertEqual((epoch, start, end, len(data)), (1, 0, 3, 3 * zs.VOXEL.size))
        self.assertEqual(hub.wait_map(1, 3, timeout=0)[1:3], (3, 3))
        hub.put_voxels(2, record)
        self.assertEqual(hub.wait_map(1, 3, timeout=0)[:3], (2, 0, 1))

    def test_image_wait_and_restart(self):
        hub = cf.CameraHub()
        hub.put_image("color", "image/jpeg", b"a")
        seq, mime, data = hub.wait_image("color", 0, timeout=0)
        self.assertEqual((mime, data), ("image/jpeg", b"a"))
        hub.reset()
        self.assertEqual(hub.wait_image("color", seq, timeout=0)[2], b"")
        hub.put_image("color", "image/jpeg", b"b")
        self.assertEqual(hub.wait_image("color", seq, timeout=0)[2], b"b")

    def test_png_encoder(self):
        data = cf.png(2, 1, bytes([255, 0, 0, 0, 0, 255]))
        self.assertTrue(data.startswith(b"\x89PNG\r\n\x1a\n"))
        self.assertIn(b"IEND", data)


class Simulator(unittest.TestCase):
    def test_sim_camera_follows_chassis_and_maps_what_it_sees(self):
        hub = cf.CameraHub()
        reports = []
        command = [(0.0, 0.0, 0.0)]
        camera = cf.SimCamera(hub, lambda kind, value: reports.append((kind, value)), lambda: (command[0], 0.3))
        try:
            camera.send({"op": "integrate", "on": True})
            end = time.monotonic() + 3
            while hub.count == 0 and time.monotonic() < end:
                time.sleep(0.05)
            self.assertGreater(hub.count, 100)
            self.assertTrue(hub.images["color"][2].startswith(b"\x89PNG"))
            command[0] = (0.0, 0.0, 0.5)
            time.sleep(0.6)
            poses = [value for kind, value in reports if kind == "pose"]
            self.assertGreater(poses[-1]["q"][2], 0.05)  # turned left (positive yaw)
            before = hub.count
            time.sleep(0.6)
            self.assertGreater(hub.count, before)
            camera.send({"op": "reset"})
            self.assertEqual(hub.count, 0)
        finally:
            camera.stop()


class CameraEndpoints(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = mc.Controller(simulation=True)
        cls.token = "map-token"
        cls.server = mc.make_server(cls.controller, 0, cls.token)
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()
        cls.controller.hub.put_voxels(1, zs.VOXEL.pack(1, 2, 3, 4, 5, 6))
        cls.controller.hub.put_image("depth", "image/png", b"png-bytes")

    @classmethod
    def tearDownClass(cls):
        cls.controller.closing = True
        cls.server.shutdown()

    def get(self, path):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=3)
        connection.request("GET", path)
        response = connection.getresponse()
        body = response.read()
        connection.close()
        return response, body

    def test_endpoints_require_token(self):
        self.assertEqual(self.get("/map?epoch=0&since=0")[0].status, 403)
        self.assertEqual(self.get("/camera?kind=color")[0].status, 403)

    def test_map_endpoint(self):
        response, body = self.get(f"/map?t={self.token}&epoch=0&since=0")
        self.assertEqual(response.status, 200)
        self.assertEqual((response.getheader("X-Map-Epoch"), response.getheader("X-Map-End")), ("1", "1"))
        self.assertEqual(body, zs.VOXEL.pack(1, 2, 3, 4, 5, 6))

    def test_camera_endpoint(self):
        response, body = self.get(f"/camera?t={self.token}&kind=depth&after=0")
        self.assertEqual((response.status, response.getheader("Content-Type"), body),
                         (200, "image/png", b"png-bytes"))
        self.assertEqual(self.get(f"/camera?t={self.token}&kind=other")[0].status, 400)


if __name__ == "__main__":
    unittest.main()
