"""ARX X7s Mac controller: a local browser page in front of the robot's SSH control bridge.

This process owns the SSH link and every input safety rule. The page only
reports which keys are held and whether it has focus. The page is served on
127.0.0.1 and every request needs this launch's random token.
"""

import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import queue
import secrets
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qs, urlparse

from camera_feed import CameraHub, RemoteCamera, SimCamera
from control_bridge import OLD_PREFIXES, PREFIX
from robot_session import ARM_SELECTIONS, DRIVE_KEYS, DRIVE_MODES, DRIVE_PHASES, JOG_KEYS, OPERATIONS, SPEED_LEVELS

ROOT = Path(__file__).resolve().parent
PAGE = ROOT / "controller_ui.html"
DEFAULT_ROBOT = "arx@10.90.134.126"
REMOTE_DIR = "$HOME/Documents/Teleop Test Enviroment"
BUSY_PREFIX = b"ARX-BUSY "
CONTROL_KEYS = set(DRIVE_KEYS) | JOG_KEYS
BROWSER_FRESH = 0.20   # browser input older than this counts as released
BROWSER_GONE = 0.50    # a silent page is treated as unfocused
HEARTBEAT_WAIT = 0.6   # stop if robot status stops arriving
STARTUP_WAIT = 6.0     # the bridge can block while it opens the body SDK
CONNECT_WAIT = 12.0
OP_RETRY = 1.0
DRIVE_LINGER = 0.5     # without Space: keep drive mode this long after the last drive key, so the base ramps down
SSH_OPTIONS = ["-o", "StrictHostKeyChecking=yes", "-o", "UpdateHostKeys=no", "-o", "ClearAllForwardings=yes",
               "-o", "ForwardAgent=no"]
MAP_MODES = ("auto", "on", "off")  # auto: build the map only in mapping mode (arms folded)
CAMERA_REMOTE = ('LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:/usr/local/zed/lib '
                 f'exec python3 -Bu "{REMOTE_DIR}/zed_stream.py"')
OP_LABELS = {"enable": "Enable", "park": "Park", "recover": "Recover", "map": "Mapping mode"}
OP_PHASES = {"enable": ("idle", "paused", "parked", "mapping"), "park": ("ready", "paused", "mapping"),
             "recover": ("fault",), "map": ("ready", "paused")}


class Controller:
    def __init__(self, robot=DEFAULT_ROBOT, socket_path=None, simulation=True, sim_stall=False):
        self.robot = robot
        self.socket_path = socket_path
        self.simulation = simulation
        self.sim_stall = sim_stall
        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)
        self.version = 0
        self.events = queue.SimpleQueue()
        self.process = None
        self.connecting = False
        self.state = {}
        self.last_status = 0.0
        self.status_times = deque(maxlen=40)
        self.waiting = False
        self.busy = None
        self.live_started = False
        self.robot_seq = 0
        self.idle_notice = None
        self.last_detail = None
        self.log = deque(maxlen=300)
        self.log_seq = 0
        self.owner = None
        self.browser_seen = 0.0
        self.focused = False
        self.keys = set()
        self.armed_keys = set()
        self.space_blocked = False
        self.require_space = False  # True restores the hold-Space deadman
        self.last_drive = -1e9
        self.pending = ""
        self.op = None
        self.speed = 0
        self.selection = "both"
        self.drive_mode = DRIVE_MODES[0]
        self.monitor_process = None
        self.monitor = None
        self.monitor_note = None
        self.monitor_starting = False
        self.hub = CameraHub()
        self.camera = None
        self.camera_starting = False
        self.camera_note = None
        self.camera_pose = None
        self.camera_saved = None
        self.map_mode = "auto"
        self.integrating = None
        self.closing = False
        self.quit_requested = threading.Event()
        self.add_log("Simulator ready. Press Connect." if simulation else f"Ready to connect to {robot}.")

    # Helpers --------------------------------------------------------------------

    def notify(self):
        with self.lock:
            self.version += 1
            self.changed.notify_all()

    def add_log(self, text, level="info", source="app"):
        with self.lock:
            self.log_seq += 1
            self.log.append({"seq": self.log_seq, "time": time.strftime("%H:%M:%S"), "level": level,
                             "text": text, "source": source})
            self.notify()

    @property
    def phase(self):
        return self.state.get("phase", "connecting" if self.process or self.connecting else "offline")

    def ssh(self, remote):
        command = ["/usr/bin/ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                   "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2", *SSH_OPTIONS]
        if self.socket_path:
            command += ["-S", self.socket_path, "-o", "ProxyCommand=/usr/bin/false"]
        return command + [self.robot, remote]

    def ensure_master(self):
        """Reopen the shared SSH session if it ended. A password prompt appears in Terminal."""
        if not self.socket_path:
            return True
        check = subprocess.run(["/usr/bin/ssh", "-S", self.socket_path, "-O", "check", self.robot],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if check.returncode == 0:
            return True
        self.add_log("The SSH session to the robot ended. Enter the robot password in the Terminal window "
                     "if it asks.", "warn")
        master = subprocess.run(["/usr/bin/ssh", "-fNT", "-M", "-S", self.socket_path, "-o", "ConnectTimeout=6",
                                 "-o", "NumberOfPasswordPrompts=1", "-o", "ServerAliveInterval=5",
                                 "-o", "ServerAliveCountMax=2", *SSH_OPTIONS, self.robot])
        if master.returncode != 0:
            self.add_log("Could not sign into the robot. Check the lab network and the robot's address.", "fault")
            return False
        return True

    def send(self, frame):
        if not self.process:
            return False
        try:
            os.write(self.process.stdin.fileno(), json.dumps(frame).encode() + b"\n")
        except (OSError, ValueError):
            return False
        return True

    # Connection -------------------------------------------------------------------

    def connect(self, takeover=False):
        with self.lock:
            if self.process or self.connecting:
                return
            self.connecting = True
            self.busy = None
            self.notify()
        threading.Thread(target=self._connect, args=(takeover,), daemon=True).start()

    def _connect(self, takeover):
        process = None
        try:
            if self.simulation:
                command = [sys.executable, "-Bu", str(ROOT / "control_bridge.py")]
                if self.sim_stall:
                    command.append("--sim-stall")
            else:
                if not self.ensure_master():
                    return
                remote = f'exec bash "{REMOTE_DIR}/run-controller.sh"' + (" --takeover" if takeover else "")
                command = self.ssh(remote)
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, bufsize=0, start_new_session=True)
            os.set_blocking(process.stdin.fileno(), False)
        except OSError as exc:
            self.add_log(f"Could not start the controller: {exc}", "fault")
            return
        finally:
            with self.lock:
                self.connecting = False
                if process is not None:
                    self.process = process
                    self.state = {}
                    self.last_status = time.monotonic()
                    self.status_times.clear()
                    self.waiting = False
                    self.robot_seq = 0
                    self.idle_notice = None
                    self.last_detail = None
                    self.block_space()
                self.notify()
        if takeover:
            self.add_log("Taking over from the previous controller session.", "warn")
        self.add_log("Connecting to the simulator." if self.simulation else f"Connecting to {self.robot}.")
        threading.Thread(target=self.read, args=(process,), daemon=True).start()

    def read(self, process):
        with process.stdout:
            for line in process.stdout:
                if line.startswith(PREFIX):
                    try:
                        self.events.put((process, "state", json.loads(line[len(PREFIX):])))
                    except ValueError:
                        pass
                elif line.startswith(OLD_PREFIXES):
                    self.events.put((process, "old", None))
                    break
                elif line.startswith(BUSY_PREFIX):
                    try:
                        busy = json.loads(line[len(BUSY_PREFIX):])
                    except ValueError:
                        busy = {"bridge": False}
                    self.events.put((process, "busy", busy))
                elif line.strip():
                    self.events.put((process, "log", line.decode(errors="replace").strip()))
        self.events.put((process, "closed", process.wait()))

    def disconnect(self, notice=None, level="info"):
        with self.lock:
            process, self.process = self.process, None
            if process:
                try:
                    os.write(process.stdin.fileno(), b'{"op": "quit"}\n')
                except OSError:
                    pass
                try:
                    process.stdin.close()
                except OSError:
                    pass
                threading.Thread(target=self.reap, args=(process,), daemon=True).start()
            self.state = {}
            self.waiting = False
            self.live_started = False
            self.op = None
            self.block_space()
            if notice:
                self.add_log(notice, level)
            self.notify()

    @staticmethod
    def reap(process):
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.terminate()

    # Robot messages ------------------------------------------------------------------

    def run(self):
        while not self.closing:
            try:
                process, kind, value = self.events.get(timeout=0.02)
            except queue.Empty:
                process = None
            with self.lock:
                if process is not None and process is self.process:
                    self.handle(kind, value)
                self.tick(time.monotonic())

    def handle(self, kind, value):
        if kind == "state":
            self.on_status(value)
        elif kind == "old":
            self.disconnect("The robot has the old controller. Run Send Controller Update.command first.", "fault")
        elif kind == "busy":
            self.busy = value
            if value.get("takeover") == "timeout":
                self.add_log("The previous controller did not stop within 8 s. Check the robot terminal.", "fault")
            elif value.get("bridge"):
                self.add_log("A previous controller session is still running on the robot and holding it. "
                             "Take over to stop it, or wait up to a minute for it to time out.", "warn")
            else:
                self.add_log("Another robot program (the jog console or an update) is using the controls. "
                             "Close it on the robot first.", "warn")
        elif kind == "log":
            self.last_detail = value[:300]
            self.add_log(self.last_detail, "detail", "robot")
        elif kind == "closed":
            if self.busy:
                self.disconnect()
            elif self.state:
                self.disconnect(f"Controller connection ended (exit {value}). If the robot is powered, "
                                "check that the arms, lift, and torso are supported.", "warn")
            else:
                self.disconnect(f"The controller did not start (exit {value}).", "fault")

    def on_status(self, status):
        now = time.monotonic()
        old_phase = self.state.get("phase")
        self.state = status
        self.last_status = now
        self.status_times.append(now)
        if self.waiting:
            self.waiting = False
            self.add_log("Robot feedback restored.")
        if status.get("event_seq", 0) < self.robot_seq:
            self.robot_seq = 0
        for seq, level, text in status.get("events", []):
            if seq > self.robot_seq:
                self.robot_seq = seq
                self.add_log(text, level, "robot")
        phase = status.get("phase")
        if phase == "idle" and status.get("notice") != self.idle_notice:
            self.idle_notice = status.get("notice")
            self.add_log(self.idle_notice, "info", "robot")
        if status.get("live") and status.get("started"):
            self.live_started = True
        if phase != old_phase:
            self.block_space()
        self.reply(status, now)
        self.notify()

    def browser_fresh(self, now):
        return self.focused and now - self.browser_seen < BROWSER_FRESH

    def deadman_ready(self, phase=None):
        """Control keys can act: the robot is drivable and, if required, Space is held."""
        if self.waiting or (phase or self.phase) not in DRIVE_PHASES:
            return False
        return not self.require_space or ("space" in self.keys and not self.space_blocked)

    def deadman(self, now, phase=None):
        if not (self.browser_fresh(now) and self.deadman_ready(phase)):
            return False
        if self.require_space:
            return True
        # Without Space the page counts as held while focused. After a stop or warning the
        # robot ignores input until it sees a release. Letting go of every control key is that
        # release: the robot gets one not-held frame only then, so a key still held cannot
        # resume motion by itself.
        if self.state.get("release_required"):
            return bool(self.keys & CONTROL_KEYS)
        return True

    def reply(self, status, now):
        phase = status.get("phase")
        frame = {"lease": status["lease"], "speed": self.speed, "arms": self.selection,
                 "drive_mode": self.drive_mode}
        if self.op and (phase != self.op[1] or now - self.op[2] > OP_RETRY):
            self.op = None
        if self.op:
            frame.update(deadman=False, keys=[], action=self.op[0])
        else:
            deadman = self.deadman(now, phase)
            drive = sorted(self.armed_keys & DRIVE_KEYS.keys()) if deadman else []
            if drive:
                self.last_drive = now
            if not self.require_space and not drive and now - self.last_drive >= DRIVE_LINGER:
                # No drive key: brake instead of holding drive mode at zero velocity.
                frame["drive_mode"] = "pulse"
            frame.update(deadman=deadman, keys=drive, action=self.pending if deadman else "")
            self.pending = ""
        self.send(frame)

    def tick(self, now):
        if self.process and self.state:
            limit = STARTUP_WAIT if self.state.get("phase") == "homing" else HEARTBEAT_WAIT
            if now - self.last_status > limit and not self.waiting:
                self.send({"op": "stop"})
                self.waiting = True
                self.block_space()
                self.add_log("Robot feedback interrupted. Stop requested.", "warn")
        elif self.process and not self.state and now - self.last_status > CONNECT_WAIT:
            self.disconnect("The controller did not respond. Check the lab connection and SDK startup.", "fault")
        self.update_integration()
        if self.focused and now - self.browser_seen > BROWSER_GONE:
            self.focused = False
            self.apply_keys(set())
            self.focus_lost("The controller page stopped responding.")

    # Operator input ------------------------------------------------------------------

    def block_space(self):
        if "space" in self.keys:
            self.space_blocked = True
        self.armed_keys = set()
        self.pending = ""
        self.last_drive = -1e9

    def apply_keys(self, keys):
        previous = self.keys
        if "space" in keys and "space" not in previous and (self.phase not in DRIVE_PHASES or self.waiting):
            self.space_blocked = True
        if "space" not in keys:
            self.space_blocked = False
        self.keys = keys
        if self.deadman_ready():
            new = keys - previous - {"space"}
            # Keys already down before control was possible count only after they are pressed again.
            self.armed_keys = (self.armed_keys | new) & keys
            for key in sorted(new):
                if key in JOG_KEYS and not self.pending:
                    self.pending = key
        else:
            self.armed_keys = set()
            self.pending = ""
        self.notify()

    def focus_lost(self, reason):
        if self.process and self.phase in DRIVE_PHASES:
            self.stop(f"{reason} Stopped.")

    def update_input(self, client, keys, focused):
        with self.lock:
            if client is None or client != self.owner:
                return False
            now = time.monotonic()
            was_focused = self.focused and now - self.browser_seen < BROWSER_GONE
            self.browser_seen = now
            self.focused = focused
            keys = {key for key in keys if isinstance(key, str) and (key in CONTROL_KEYS or key == "space")}
            if not focused:
                keys = set()
            if was_focused and not focused:
                self.apply_keys(set())
                self.focus_lost("The controller page lost focus.")
            self.apply_keys(keys)
            return True

    def stop(self, reason="Stop requested."):
        with self.lock:
            self.op = None
            self.block_space()
            if self.send({"op": "stop"}):
                self.add_log(reason, "warn")
            self.notify()

    def command(self, client, op, value=None):
        with self.lock:
            if op == "claim":
                if client != self.owner:
                    self.owner = client
                    self.focused = False
                    self.apply_keys(set())
                return {"ok": True}
            if client is None or client != self.owner:
                return {"ok": False, "error": "Another browser tab has control."}
            phase = self.phase
            if op == "connect":
                self.connect()
            elif op == "takeover":
                self.connect(takeover=True)
            elif op == "disconnect":
                self.disconnect("Disconnected.")
            elif op == "stop":
                self.stop()
            elif op in OPERATIONS:
                if not self.process or phase not in OP_PHASES[op] or self.waiting:
                    return {"ok": False, "error": f"{OP_LABELS[op]} is not available now."}
                self.op = (op, phase, time.monotonic())
                self.block_space()
                self.add_log(f"{OP_LABELS[op]} requested.")
            elif op == "speed":
                if type(value) is not int or not 0 <= value < len(SPEED_LEVELS):
                    return {"ok": False, "error": "Unknown speed."}
                self.speed = value
            elif op == "arms":
                if value not in ARM_SELECTIONS:
                    return {"ok": False, "error": "Unknown arm selection."}
                self.selection = value
            elif op == "require_space":
                self.require_space = value is True
                self.block_space()
                self.add_log("Space must be held for every control." if self.require_space else
                             "Space is no longer required. Keys act while this page has focus.")
            elif op == "drive_mode":
                if value not in DRIVE_MODES:
                    return {"ok": False, "error": "Unknown brake setting."}
                self.drive_mode = value
                self.add_log("Chassis brakes when Space is released." if value == "latched" else
                             "Chassis brakes whenever no drive key is held.")
            elif op == "monitor":
                self.set_monitor(bool(value))
            elif op == "camera":
                self.set_camera(bool(value))
            elif op == "map_mode":
                if value not in MAP_MODES:
                    return {"ok": False, "error": "Unknown map setting."}
                self.map_mode = value
                self.update_integration()
            elif op in ("map_reset", "map_save"):
                if not self.camera:
                    return {"ok": False, "error": "Start the camera first."}
                self.camera.send({"op": "reset" if op == "map_reset" else "save"})
            elif op == "quit":
                self.quit_requested.set()
            else:
                return {"ok": False, "error": "Unknown command."}
            self.notify()
            return {"ok": True}

    # CAN monitor ------------------------------------------------------------------------

    def set_monitor(self, on):
        if not on:
            self.monitor_starting = False
            process, self.monitor_process = self.monitor_process, None
            if process:
                process.terminate()
            self.monitor = None
            self.monitor_note = None
            return
        if self.monitor_process:
            return
        if self.simulation:
            self.monitor_note = "The CAN monitor reads the robot's can5 bus. It is unavailable in the simulator."
            return
        self.monitor_note = "Starting the read-only can5 monitor…"
        self.monitor_starting = True
        threading.Thread(target=self._monitor, daemon=True).start()

    def _monitor(self):
        if not self.ensure_master():
            with self.lock:
                self.monitor_starting = False
                self.monitor_note = "Could not reach the robot."
                self.notify()
            return
        process = subprocess.Popen(self.ssh(f'exec python3 -Bu "{REMOTE_DIR}/can_monitor.py"'),
                                   stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        with self.lock:
            self.monitor_starting = False
            if self.closing:
                process.terminate()
                return
            self.monitor_process = process
        with process.stdout:
            for line in process.stdout:
                with self.lock:
                    if self.monitor_process is not process:
                        break
                    try:
                        self.monitor = json.loads(line)
                        self.monitor_note = None
                    except ValueError:
                        self.monitor_note = line.decode(errors="replace").strip()[:200]
                    self.notify()
        process.wait()
        with self.lock:
            if self.monitor_process is process:
                self.monitor_process = None
                self.monitor_note = f"Monitor stopped (exit {process.returncode})."
                self.notify()

    # Camera and live map -------------------------------------------------------------------

    def mapping_pose(self):
        """True while the arms are folded for mapping, including a pause taken in mapping mode."""
        phase = self.state.get("phase")
        return phase == "mapping" or phase == "paused" and self.state.get("resume_phase") == "mapping"

    def wants_integration(self):
        return bool(self.camera) and (self.map_mode == "on" or self.map_mode == "auto" and self.mapping_pose())

    def update_integration(self):
        wanted = self.wants_integration()
        if self.camera and wanted != self.integrating:
            self.integrating = wanted
            self.camera.send({"op": "integrate", "on": wanted})

    def sim_robot_state(self):
        with self.lock:
            command = self.state.get("command") or (0.0, 0.0, 0.0)
            return tuple(command), (self.state.get("body") or {}).get("head")

    def set_camera(self, on):
        if not on:
            self.camera_starting = False
            camera, self.camera = self.camera, None
            if camera:
                camera.stop()
            self.camera_pose = None
            self.integrating = None
            self.hub.reset()
            self.camera_note = None
            return
        if self.camera or self.camera_starting:
            return
        if self.simulation:
            self.camera = SimCamera(self.hub, self.camera_report, self.sim_robot_state)
            self.integrating = None
            self.update_integration()
            return
        self.camera_starting = True
        self.camera_note = "Starting the ZED Mini on the robot…"
        threading.Thread(target=self._start_camera, daemon=True).start()

    def _start_camera(self):
        reachable = self.ensure_master()
        with self.lock:
            if not self.camera_starting or self.closing:
                self.camera_starting = False
                return
            self.camera_starting = False
            if not reachable:
                self.camera_note = "Could not reach the robot."
                self.notify()
                return
            try:
                self.camera = RemoteCamera(self.ssh(CAMERA_REMOTE), self.hub, self.camera_report)
            except OSError as exc:
                self.camera_note = f"Could not start the camera stream: {exc}"
                self.notify()
                return
            self.integrating = None
            self.update_integration()
            self.notify()

    def camera_report(self, kind, value):
        with self.lock:
            if kind == "pose":
                self.camera_pose = value
                self.hub.put_pose(value)
                self.camera_note = None
            elif kind == "status":
                level = value.get("level", "info")
                if value.get("saved"):
                    self.camera_saved = value["saved"]
                if level != "detail":
                    self.camera_note = value.get("text")
                self.add_log(value.get("text", ""), level, "camera")
            elif kind == "exit":
                source, code = value
                if source is self.camera:
                    self.camera = None
                    self.camera_pose = None
                    self.integrating = None
                    self.camera_note = f"Camera stream stopped (exit {code}). " + (self.camera_note or "")
            self.notify()

    def camera_snapshot(self):
        pose = self.camera_pose
        return {"on": bool(self.camera) or self.camera_starting, "note": self.camera_note, "pose": pose,
                "map_mode": self.map_mode, "integrating": bool(pose and pose.get("integrating")),
                "wants_integration": self.wants_integration(), "voxels": self.hub.count,
                "saved": self.camera_saved}

    # Page state ---------------------------------------------------------------------------

    def snapshot(self, since):
        now = time.monotonic()
        times = self.status_times
        rate = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else None
        robot = {key: value for key, value in self.state.items() if key != "events"}
        return {
            "phase": "interrupted" if self.waiting and self.state else self.phase,
            "app": {
                "simulation": self.simulation, "robot": self.robot, "connected": bool(self.process),
                "connecting": self.connecting, "waiting": self.waiting, "busy": self.busy,
                "live_started": self.live_started, "keys": sorted(self.keys), "armed_keys": sorted(self.armed_keys),
                "deadman": self.deadman(now), "space_blocked": self.space_blocked,
                "require_space": self.require_space,
                "focused": self.focused, "owner": self.owner, "speed": self.speed, "selection": self.selection,
                "drive_mode": self.drive_mode, "pending_op": self.op[0] if self.op else None,
                "status_hz": None if rate is None else round(rate, 1),
                "status_age": round(now - self.last_status, 2) if self.state else None,
                "monitor_on": bool(self.monitor_process) or self.monitor_starting,
                "monitor": self.monitor, "monitor_note": self.monitor_note,
                "camera": self.camera_snapshot(),
            },
            "robot": robot,
            "log": [entry for entry in self.log if entry["seq"] > since],
            "log_seq": self.log_seq,
        }

    def shutdown(self):
        with self.lock:
            self.closing = True
            self.set_monitor(False)
            self.set_camera(False)
            self.disconnect("Controller app closed.")
            self.notify()


def number(query, name):
    try:
        return max(0, int(query.get(name, ["0"])[0]))
    except ValueError:
        return 0


def make_server(controller, port, token):
    page = PAGE

    class Handler(BaseHTTPRequestHandler):
        server_version = "ARXController/3"
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def host_ok(self):
            return self.headers.get("Host", "") in self.server.allowed_hosts

        def respond(self, code, body, content_type="application/json", extra=()):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            for name, value in extra:
                self.send_header(name, value)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            query = parse_qs(url.query)
            if not self.host_ok() or query.get("t", [""])[0] != token:
                return self.respond(403, b"Forbidden. Reopen the controller from its launcher.", "text/plain")
            if url.path == "/":
                policy = ("default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; "
                          "connect-src 'self'; img-src data: blob:; frame-ancestors 'none'; base-uri 'none'")
                return self.respond(200, page.read_bytes(), "text/html; charset=utf-8",
                                    (("Content-Security-Policy", policy), ("Referrer-Policy", "no-referrer")))
            if url.path == "/events":
                try:
                    since = int(query.get("since", ["0"])[0])
                except ValueError:
                    since = 0
                return self.stream(since)
            if url.path == "/camera":
                kind = query.get("kind", ["color"])[0]
                if kind not in ("color", "depth"):
                    return self.respond(400, b"Unknown view", "text/plain")
                seq, mime, data = controller.hub.wait_image(kind, number(query, "after"))
                if not data or seq <= number(query, "after"):
                    return self.respond(204, b"", "text/plain")
                return self.respond(200, data, mime, (("X-Frame-Seq", str(seq)),))
            if url.path == "/map":
                epoch, start, end, data = controller.hub.wait_map(number(query, "epoch"), number(query, "since"))
                return self.respond(200, data, "application/octet-stream",
                                    (("X-Map-Epoch", str(epoch)), ("X-Map-Start", str(start)),
                                     ("X-Map-End", str(end))))
            return self.respond(404, b"Not found", "text/plain")

        def do_POST(self):
            if not self.host_ok() or self.headers.get("X-ARX-Token") != token:
                return self.respond(403, b'{"ok": false, "error": "Forbidden"}')
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096:
                    raise ValueError
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError
            except ValueError:
                return self.respond(400, b'{"ok": false, "error": "Bad request"}')
            client = body.get("client") if isinstance(body.get("client"), str) else None
            if self.path == "/api/input":
                keys = body.get("keys") if isinstance(body.get("keys"), list) else []
                result = {"ok": controller.update_input(client, keys[:32], body.get("focused") is True)}
            elif self.path == "/api/command":
                result = controller.command(client, body.get("op"), body.get("value"))
            else:
                return self.respond(404, b'{"ok": false}')
            return self.respond(200, json.dumps(result).encode())

        def stream(self, since):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            version = -1
            try:
                while not controller.closing:
                    with controller.changed:
                        controller.changed.wait_for(lambda: controller.version != version or controller.closing,
                                                    timeout=1.0)
                        version = controller.version
                        data = controller.snapshot(since)
                        since = data["log_seq"]
                    self.wfile.write(b"data: " + json.dumps(data, allow_nan=False).encode() + b"\n\n")
                    self.wfile.flush()
                    time.sleep(0.02)
                self.wfile.write(b"event: closed\ndata: {}\n\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ValueError):
                pass

    server = None
    for candidate in (port, 0):
        try:
            server = ThreadingHTTPServer(("127.0.0.1", candidate), Handler)
            break
        except OSError:
            continue
    if server is None:
        raise OSError("Could not open a local port for the controller page")
    server.daemon_threads = True
    bound = server.server_address[1]
    server.allowed_hosts = {f"127.0.0.1:{bound}", f"localhost:{bound}"}
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", default=os.environ.get("ARX_SSH_SOCKET"),
                        help="Shared SSH control socket from the launcher")
    parser.add_argument("--robot", default=os.environ.get("ARX_ROBOT", DEFAULT_ROBOT))
    parser.add_argument("--simulation", action="store_true")
    parser.add_argument("--sim-stall", action="store_true", help="Simulator: wheels never report motion")
    parser.add_argument("--connect", action="store_true", help="Connect without a control socket")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    simulation = args.simulation or not (args.socket or args.connect)
    controller = Controller(args.robot, args.socket, simulation, args.sim_stall)
    token = secrets.token_urlsafe(18)
    server = make_server(controller, args.port, token)
    url = f"http://127.0.0.1:{server.server_address[1]}/?t={token}"
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, lambda *_: controller.quit_requested.set())
    threading.Thread(target=controller.run, daemon=True).start()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"ARX controller page: {url}", flush=True)
    print("Keep this window open while using the robot. Closing it disconnects the controller.", flush=True)
    if not args.no_browser:
        subprocess.run(["/usr/bin/open", url], check=False)
    try:
        while not controller.quit_requested.wait(0.5):
            pass
    finally:
        controller.shutdown()
        time.sleep(0.3)
        server.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
