"""Camera and live map plumbing for the Mac dashboard.

CameraHub keeps the latest color and depth frames, the camera pose, and an
append-only log of voxel records for the page. RemoteCamera runs zed_stream.py
on the robot over SSH. SimCamera is a pure-Python stand-in (a furnished room
seen from a simulated head camera that follows the simulator's chassis
commands) so the dashboard can be tried without the robot.
"""

import json
import math
import struct
import subprocess
import threading
import time
import zlib

from zed_stream import EPOCH, VOXEL, VOXEL_SIZE, read_messages, unpack_voxels

MAP_CHUNK = 150_000  # records per page request


class CameraHub:
    def __init__(self):
        self.cond = threading.Condition()
        self.images = {"color": (0, None, b""), "depth": (0, None, b"")}
        self.image_seq = 0
        self.epoch = 1
        self.records = bytearray()
        self.pose = None
        self.pose_time = 0.0
        self.wanted = {"color": 0.0, "depth": 0.0}

    @property
    def count(self):
        return len(self.records) // VOXEL.size

    def put_image(self, kind, mime, data):
        with self.cond:
            self.image_seq += 1
            self.images[kind] = (self.image_seq, mime, bytes(data))
            self.cond.notify_all()

    def put_voxels(self, epoch, data):
        with self.cond:
            if epoch != self.epoch:
                self.epoch = epoch
                self.records = bytearray()
            self.records += data
            self.cond.notify_all()

    def put_pose(self, pose):
        with self.cond:
            self.pose = pose
            self.pose_time = time.monotonic()

    def reset(self):
        with self.cond:
            # Sequence numbers keep counting so a page waiting for "after N" sees the next stream.
            self.images = {kind: (self.image_seq, None, b"") for kind in self.images}
            self.pose = None
            self.cond.notify_all()

    def wants(self, kind, within=2.0):
        return time.monotonic() - self.wanted[kind] < within

    def wait_image(self, kind, after, timeout=1.0):
        with self.cond:
            self.wanted[kind] = time.monotonic()
            self.cond.wait_for(lambda: self.images[kind][0] > after, timeout)
            return self.images[kind]

    def wait_map(self, epoch, since, timeout=1.0):
        with self.cond:
            if epoch == self.epoch and since <= self.count:
                self.cond.wait_for(lambda: self.count > since or self.epoch != epoch, timeout)
            if epoch != self.epoch or since > self.count:
                since = 0
            end = min(self.count, since + MAP_CHUNK)
            return self.epoch, since, end, bytes(self.records[since * VOXEL.size:end * VOXEL.size])


class RemoteCamera:
    """zed_stream.py on the robot through the shared SSH session."""

    def __init__(self, command, hub, report):
        self.hub = hub
        self.report = report
        self.write_lock = threading.Lock()
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, start_new_session=True)
        threading.Thread(target=self.read, daemon=True).start()
        threading.Thread(target=self.read_errors, daemon=True).start()

    def send(self, message):
        with self.write_lock:
            try:
                self.process.stdin.write(json.dumps(message).encode() + b"\n")
                self.process.stdin.flush()
            except (OSError, ValueError):
                pass

    def read(self):
        try:
            for kind, payload in read_messages(self.process.stdout):
                if kind in ("color", "depth"):
                    self.hub.put_image(kind, "image/jpeg", payload)
                    self.send({"op": "ack", "kind": kind})
                elif kind == "voxels":
                    self.hub.put_voxels(*unpack_voxels(payload))
                elif kind == "pose":
                    self.report("pose", json.loads(payload))
                elif kind == "status":
                    self.report("status", json.loads(payload))
        except (OSError, ValueError) as exc:
            self.report("status", {"level": "warn", "text": f"Camera stream error: {exc}"})
        self.report("exit", (self, self.process.wait()))

    def read_errors(self):
        for line in self.process.stderr:
            text = line.decode(errors="replace").strip()
            if text:
                self.report("status", {"level": "detail", "text": text[:300]})

    def stop(self):
        self.send({"op": "quit"})
        try:
            self.process.stdin.close()
        except OSError:
            pass
        threading.Thread(target=self._reap, daemon=True).start()

    def _reap(self):
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.terminate()


# Simulator -------------------------------------------------------------------------

CAMERA_HEIGHT = 1.35   # simulated camera height above the floor, m
CAMERA_FORWARD = 0.10  # camera ahead of the base center, m
SIM_W, SIM_H, SIM_F = 160, 90, 80.0  # image size and focal length in pixels (90° wide)
MIN_RANGE, MAX_RANGE = 0.45, 4.0


def png(width, height, rgb):
    """Encode packed RGB bytes as a PNG with the standard library."""
    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data))
    stride = width * 3
    raw = b"".join(b"\x00" + bytes(rgb[row * stride:(row + 1) * stride]) for row in range(height))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 3)) + chunk(b"IEND", b""))


TURBO = ((48, 18, 59), (70, 134, 251), (27, 229, 181), (164, 252, 60), (251, 185, 56), (228, 70, 10), (122, 4, 3))


def turbo(value):
    value = min(1.0, max(0.0, value)) * (len(TURBO) - 1)
    i = min(int(value), len(TURBO) - 2)
    t = value - i
    a, b = TURBO[i], TURBO[i + 1]
    return tuple(int(a[c] + (b[c] - a[c]) * t) for c in range(3))


def build_room():
    """Voxels (x, y, z, r, g, b, key) of a furnished room on a 10 cm grid, ZED world frame."""
    floor = -CAMERA_HEIGHT
    cells = {}

    def add(x, y, z, color, shade=1.0):
        ix, iy, iz = (math.floor(v / VOXEL_SIZE) for v in (x, y, z))
        noise = ((ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791)) % 23 - 11
        rgb = tuple(max(0, min(255, int(c * shade) + noise)) for c in color)
        cells[(ix, iy, iz)] = rgb

    def box(x0, x1, y0, y1, z0, z1, color, step=0.1, top_only=False):
        xs = [x0 + i * step for i in range(int(round((x1 - x0) / step)) + 1)]
        ys = [y0 + i * step for i in range(int(round((y1 - y0) / step)) + 1)]
        zs = [z0 + i * step for i in range(int(round((z1 - z0) / step)) + 1)]
        for x in xs:
            for y in ys:
                add(x, y, z1, color, 1.0)
        if top_only:
            return
        for x in xs:
            for z in zs:
                add(x, y0, z, color, 0.8)
                add(x, y1, z, color, 0.8)
        for y in ys:
            for z in zs:
                add(x0, y, z, color, 0.65)
                add(x1, y, z, color, 0.65)

    x0, x1, y0, y1, height = -2.5, 5.5, -3.0, 3.0, 2.4
    for i in range(int((x1 - x0) / 0.1) + 1):
        for j in range(int((y1 - y0) / 0.1) + 1):
            x, y = x0 + i * 0.1, y0 + j * 0.1
            tile = (int(math.floor(x / 0.5)) + int(math.floor(y / 0.5))) % 2
            add(x, y, floor, (150, 128, 100) if tile else (128, 108, 84))
    walls = {"north": (196, 204, 182), "south": (210, 196, 170), "east": (140, 160, 178), "west": (186, 120, 96)}
    for k in range(int(height / 0.1) + 1):
        z = floor + k * 0.1
        stripe = 0.85 if abs(z - floor - 0.9) < 0.06 else 1.0
        for i in range(int((x1 - x0) / 0.1) + 1):
            x = x0 + i * 0.1
            add(x, y1, z, walls["north"], stripe)
            add(x, y0, z, walls["south"], stripe)
        for j in range(int((y1 - y0) / 0.1) + 1):
            y = y0 + j * 0.1
            add(x1, y, z, walls["east"], stripe)
            add(x0, y, z, walls["west"], stripe)
    box(2.2, 3.4, 0.9, 1.7, floor + 0.7, floor + 0.75, (120, 84, 52), top_only=True)  # table top
    for lx, ly in ((2.25, 0.95), (3.35, 0.95), (2.25, 1.65), (3.35, 1.65)):
        box(lx - 0.05, lx + 0.05, ly - 0.05, ly + 0.05, floor, floor + 0.7, (90, 62, 40))
    box(4.7, 5.3, -2.8, -1.2, floor, floor + 1.8, (70, 96, 150))       # shelf
    box(1.2, 1.7, -1.9, -1.4, floor, floor + 0.5, (220, 132, 48))      # crate
    box(0.4, 0.8, 1.8, 2.2, floor, floor + 1.0, (90, 150, 90))          # cabinet
    box(-1.6, -1.2, -2.4, -2.0, floor, floor + 1.2, (160, 160, 168))   # column
    room = []
    for (ix, iy, iz), rgb in cells.items():
        key = ((ix + 32768) << 32) | ((iy + 32768) << 16) | (iz + 32768)
        room.append(((ix + 0.5) * VOXEL_SIZE, (iy + 0.5) * VOXEL_SIZE, (iz + 0.5) * VOXEL_SIZE, rgb, key))
    return room


class SimCamera:
    """Simulated head camera. `robot_state()` returns (chassis command x, y, yaw) and head pitch."""

    def __init__(self, hub, report, robot_state):
        self.hub = hub
        self.report = report
        self.robot_state = robot_state
        self.room = build_room()
        self.seen = {}
        self.epoch = hub.epoch + 1
        self.integrate = False
        self.base = [-CAMERA_FORWARD, 0.0]
        self.yaw = 0.0
        self.stopping = threading.Event()
        threading.Thread(target=self.run, daemon=True).start()

    def send(self, message):
        op = message.get("op")
        if op == "integrate":
            self.integrate = message.get("on") is True
        elif op == "reset":
            self.seen = {}
            self.epoch += 1
            self.hub.put_voxels(self.epoch, b"")
            self.report("status", {"level": "info", "text": "Map cleared."})
        elif op == "save":
            self.report("status", {"level": "info", "text": "The simulator does not save maps. On the robot, "
                                                            "Save writes a .ply file in ~/zed-setup/maps."})

    def stop(self):
        self.stopping.set()

    def camera_pose(self, pitch):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        position = (self.base[0] + c * CAMERA_FORWARD, self.base[1] + s * CAMERA_FORWARD, 0.0)
        half_yaw, half_pitch = self.yaw / 2, pitch / 2
        cz, sz, cy, sy = math.cos(half_yaw), math.sin(half_yaw), math.cos(half_pitch), math.sin(half_pitch)
        quaternion = (-sz * sy, cz * sy, sz * cy, cz * cy)  # yaw about z, then pitch about y: x, y, z, w
        return position, quaternion

    def run(self):
        self.hub.put_voxels(self.epoch, b"")
        self.report("status", {"level": "info", "text": "Simulated camera started: a furnished room around the "
                                                        "simulated robot. Drive to map it."})
        last = time.monotonic()
        next_pose = next_frame = 0.0
        while not self.stopping.wait(1 / 30):
            now = time.monotonic()
            dt, last = min(0.1, now - last), now
            command, head = self.robot_state()
            vx, vy, wz = command
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.base[0] += (c * vx - s * vy) * dt
            self.base[1] += (s * vx + c * vy) * dt
            self.yaw += wz * dt
            pitch = 0.25 + (head - 0.30 if head is not None else 0.0)
            position, quaternion = self.camera_pose(pitch)
            if now >= next_pose:
                next_pose = now + 1 / 15
                self.report("pose", {"p": list(position), "q": list(quaternion), "tracking": "OK", "fps": 30.0,
                                     "integrating": self.integrate, "voxels": len(self.seen), "epoch": self.epoch,
                                     "simulated": True})
            if now >= next_frame and (self.integrate or self.hub.wants("color") or self.hub.wants("depth")):
                next_frame = now + 0.25
                self.frame(position, pitch)

    def frame(self, cam, pitch):
        cy_, sy_ = math.cos(self.yaw), math.sin(self.yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        forward = (cy_ * cp, sy_ * cp, -sp)
        left = (-sy_, cy_, 0.0)
        up = (cy_ * sp, sy_ * sp, cp)
        size = SIM_W * SIM_H
        zbuf = [math.inf] * size
        color = bytearray(b"\x16\x1c\x22" * size)
        visible = []
        cx, cyc = SIM_W / 2, SIM_H / 2
        for voxel in self.room:
            dx, dy, dz = voxel[0] - cam[0], voxel[1] - cam[1], voxel[2] - cam[2]
            depth = dx * forward[0] + dy * forward[1] + dz * forward[2]
            if depth < 0.15 or depth > 9.0:
                continue
            u = (dx * left[0] + dy * left[1]) / depth
            w = (dx * up[0] + dy * up[1] + dz * up[2]) / depth
            px, py = int(cx - u * SIM_F), int(cyc - w * SIM_F)
            if not (-4 <= px < SIM_W + 4 and -4 <= py < SIM_H + 4):
                continue
            splat = min(6, int(0.1 * SIM_F / depth) + 2)
            fog = max(0.35, 1.0 - depth / 12)
            rgb = bytes(int(v * fog) for v in voxel[3])
            hit = False
            for yy in range(py, py + splat):
                if not 0 <= yy < SIM_H:
                    continue
                row = yy * SIM_W
                for xx in range(px, px + splat):
                    if 0 <= xx < SIM_W and depth < zbuf[row + xx]:
                        zbuf[row + xx] = depth
                        color[3 * (row + xx):3 * (row + xx) + 3] = rgb
                        hit = True
            if hit and MIN_RANGE <= depth <= MAX_RANGE and 0 <= px < SIM_W and 0 <= py < SIM_H:
                visible.append((voxel, py * SIM_W + px, depth))
        self.hub.put_image("color", "image/png", png(SIM_W, SIM_H, color))
        depth_rgb = bytearray(3 * size)
        for i, depth in enumerate(zbuf):
            if depth < MAX_RANGE + 1:
                depth_rgb[3 * i:3 * i + 3] = bytes(turbo(1 - (depth - 0.3) / (MAX_RANGE - 0.3)))
        self.hub.put_image("depth", "image/png", png(SIM_W, SIM_H, depth_rgb))
        if not self.integrate:
            return
        records = bytearray()
        for voxel, pixel, depth in visible:
            if depth > zbuf[pixel] + 0.12:
                continue  # hidden behind something closer
            count = self.seen.get(voxel[4], 0) + 1
            self.seen[voxel[4]] = count
            if count == 2:
                key = voxel[4]
                records += VOXEL.pack((key >> 32) - 32768, ((key >> 16) & 0xffff) - 32768,
                                      (key & 0xffff) - 32768, *voxel[3])
        if records:
            self.hub.put_voxels(self.epoch, bytes(records))


__all__ = ["CameraHub", "RemoteCamera", "SimCamera", "EPOCH", "png"]
