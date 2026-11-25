"""ZED Mini color, depth, pose, and a live voxel map for the Mac dashboard.

Runs on the robot with the system Python and the ZED SDK's pyzed (not the
teleop .deps). The Mac starts it over SSH and reads binary messages from
standard output; it sends JSON commands on standard input. It exits when
standard input closes, which releases the camera.

Messages: HEADER (magic, kind, length) then the payload.
  C  color JPEG          D  depth JPEG (turbo colormap)
  P  pose JSON           V  voxel update: epoch (u32) then VOXEL records
  S  status JSON {"level", "text", ...}
Commands: {"op": "integrate", "on": bool}, {"op": "ack", "kind": "color"|"depth"},
  {"op": "reset"}, {"op": "save"}, {"op": "quit"}.

The world frame is the ZED's: gravity aligned, z up, x forward, origin where
tracking started. Voxels are integer cell indices of VOXEL_SIZE meters.
"""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import select
import struct
import sys
import time

MAGIC = b"ZED1"
HEADER = struct.Struct(">4sBI")
VOXEL = struct.Struct("<hhhBBB")
EPOCH = struct.Struct(">I")
VOXEL_SIZE = 0.025
MAX_PAYLOAD = 8 << 20
KINDS = {"color": ord("C"), "depth": ord("D"), "pose": ord("P"), "voxels": ord("V"), "status": ord("S")}
NAMES = {code: name for name, code in KINDS.items()}
IMAGE_CREDITS = 2      # images in flight per stream, so video never queues ahead of control traffic
CREDIT_TIMEOUT = 3.0
CONFIRM_FRAMES = 2     # a voxel must be seen in this many frames before it is sent
REFINE_FRAMES = (8, 32)  # resend the averaged color at these counts
LOCK_PATH = "/dev/shm/arx-zed.lock"
MAP_TURN_PAUSE_DPS = 45.0
MAP_TURN_RESUME_DPS = 25.0
MAP_TURN_SETTLE_SECONDS = 1.0
# ZED reports LOOP_CLOSED only around the frame of its correction, which can
# land a frame before or after the pose jump it causes.
LOOP_CLOSURE_WINDOW = 0.5


def pack(kind, payload):
    return HEADER.pack(MAGIC, KINDS[kind], len(payload)) + payload


def read_messages(stream):
    """Yield (kind, payload) from a binary stream until it ends."""
    while True:
        header = stream.read(HEADER.size)
        if len(header) < HEADER.size:
            return
        magic, code, length = HEADER.unpack(header)
        if magic != MAGIC or code not in NAMES or length > MAX_PAYLOAD:
            raise ValueError("Camera stream is out of sync")
        payload = stream.read(length)
        if len(payload) < length:
            return
        yield NAMES[code], payload


def unpack_voxels(payload):
    (epoch,) = EPOCH.unpack_from(payload)
    return epoch, payload[EPOCH.size:EPOCH.size + (len(payload) - EPOCH.size) // VOXEL.size * VOXEL.size]


class VoxelMap:
    """Colored occupancy cells keyed by packed int16 indices. Needs NumPy."""

    def __init__(self, np, size=VOXEL_SIZE):
        self.np = np
        self.size = size
        self.cells = {}
        self.epoch = 1

    def clear(self):
        self.cells = {}
        self.epoch += 1

    def add(self, xyz, rgb, origin, min_range, max_range):
        """Add one frame of world points. Returns packed VOXEL records for new or refined cells."""
        np = self.np
        distance = np.linalg.norm(xyz - np.asarray(origin, dtype=np.float32), axis=1)
        keep = np.isfinite(distance) & (distance >= min_range) & (distance <= max_range)
        xyz, rgb = xyz[keep], rgb[keep].astype(np.float32)
        if not len(xyz):
            return b""
        index = np.floor(xyz / self.size).astype(np.int64)
        inside = np.all(np.abs(index) < 32000, axis=1)
        index, rgb = index[inside], rgb[inside]
        keys = ((index[:, 0] + 32768) << 32) | ((index[:, 1] + 32768) << 16) | (index[:, 2] + 32768)
        unique, inverse, counts = np.unique(keys, return_inverse=True, return_counts=True)
        sums = np.stack([np.bincount(inverse, weights=rgb[:, c], minlength=len(unique)) for c in range(3)], axis=1)
        means = sums / counts[:, None]
        emitted, colors = [], []
        cells = self.cells
        for key, color in zip(unique.tolist(), means.tolist()):
            cell = cells.get(key)
            if cell is None:
                cell = cells[key] = [0, 0.0, 0.0, 0.0]
            cell[0] += 1
            cell[1] += color[0]
            cell[2] += color[1]
            cell[3] += color[2]
            if cell[0] == CONFIRM_FRAMES or cell[0] in REFINE_FRAMES:
                emitted.append(key)
                colors.append((cell[1] / cell[0], cell[2] / cell[0], cell[3] / cell[0]))
        return self.records(emitted, colors)

    def records(self, keys, colors):
        np = self.np
        if not keys:
            return b""
        keys = np.asarray(keys, dtype=np.int64)
        out = np.empty(len(keys), dtype=[("x", "<i2"), ("y", "<i2"), ("z", "<i2"),
                                         ("r", "u1"), ("g", "u1"), ("b", "u1")])
        out["x"] = (keys >> 32) - 32768
        out["y"] = ((keys >> 16) & 0xffff) - 32768
        out["z"] = (keys & 0xffff) - 32768
        rgb = np.clip(np.asarray(colors), 0, 255).astype(np.uint8)
        out["r"], out["g"], out["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        return out.tobytes()

    def confirmed(self):
        keys = [key for key, cell in self.cells.items() if cell[0] >= CONFIRM_FRAMES]
        colors = [tuple(value / self.cells[key][0] for value in self.cells[key][1:]) for key in keys]
        return keys, colors

    def save_ply(self, path):
        keys, colors = self.confirmed()
        records = self.records(keys, colors)
        np = self.np
        data = np.frombuffer(records, dtype=[("x", "<i2"), ("y", "<i2"), ("z", "<i2"),
                                             ("r", "u1"), ("g", "u1"), ("b", "u1")])
        vertices = np.empty(len(data), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                              ("r", "u1"), ("g", "u1"), ("b", "u1")])
        for axis in "xyz":
            vertices[axis] = (data[axis].astype(np.float32) + 0.5) * self.size
        for channel in "rgb":
            vertices[channel] = data[channel]
        header = ("ply\nformat binary_little_endian 1.0\n"
                  f"comment ARX X7s ZED Mini voxel map, {self.size} m cells, ZED world frame (z up)\n"
                  f"element vertex {len(vertices)}\n"
                  "property float x\nproperty float y\nproperty float z\n"
                  "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as handle:
            handle.write(header.encode())
            handle.write(vertices.tobytes())
        return len(vertices)


class MappingPoseGuard:
    """Reject discontinuities that exceed plausible motion between ZED poses."""

    def __init__(self):
        self.previous = None
        self.fault = None
        self.turn_until = 0.0
        self.anchor = None
        self.mapped_views = deque(maxlen=512)
        self.recovery_still_pose = None
        self.loop_closed = False          # a loop closure was seen since the last untrusted jump
        self.loop_closed_at = -math.inf
        self.fault_previous = None        # last pose seen while faulted, to catch new jumps

    def note_loop_closure(self, now, loop_closed):
        """Record ZED's LOOP_CLOSED status. Call on every frame, whatever the tracking state."""
        if loop_closed:
            self.loop_closed_at = now
            if self.fault:
                self.loop_closed = True

    def recent_loop_closure(self, now):
        return abs(now - self.loop_closed_at) <= LOOP_CLOSURE_WINDOW

    @staticmethod
    def jumped(elapsed, distance, angle):
        dt = min(max(0.0, elapsed), 0.25)
        return distance > 0.20 + 2.0 * dt or angle > 20.0 + 150.0 * dt

    def observe_rotation(self, now, rate):
        if rate is None:
            return
        threshold = MAP_TURN_RESUME_DPS if now < self.turn_until else MAP_TURN_PAUSE_DPS
        if rate >= threshold:
            if not self.turning(now) and not self.fault:
                self.anchor = self.previous
            self.turn_until = now + MAP_TURN_SETTLE_SECONDS

    def turning(self, now):
        return now < self.turn_until

    @staticmethod
    def angle_between(q, r):
        # q and -q represent the same rotation.
        dot = min(1.0, abs(sum(a * b for a, b in zip(q, r))))
        return math.degrees(2.0 * math.acos(dot))

    def remember_mapped_view(self):
        if self.previous is None:
            return
        _, position, q, _ = self.previous
        if not any(math.dist(position, p) < 0.35 and self.angle_between(q, r) < 18.0
                   for p, r in self.mapped_views):
            self.mapped_views.append((position, q))

    def try_relocalize(self, now, position, q, gyro_dps):
        # A jump during the fault is either ZED's loop-closure correction or
        # fresh drift. Fresh drift voids any earlier recognition.
        if self.fault_previous is not None:
            then, p0, q0 = self.fault_previous
            if self.jumped(now - then, math.dist(position, p0), self.angle_between(q, q0)):
                self.loop_closed = self.recent_loop_closure(now)
                self.recovery_still_pose = None
        self.fault_previous = (now, position, q)
        if not self.loop_closed:
            return
        candidates = list(self.mapped_views)
        if not candidates and self.anchor is not None:
            _, anchor_position, anchor_q, _ = self.anchor
            candidates.append((anchor_position, anchor_q))
        near_anchor = any(math.dist(position, p) <= 0.25 and self.angle_between(q, r) <= 15.0
                          for p, r in candidates)
        if not near_anchor or gyro_dps is None or gyro_dps >= MAP_TURN_RESUME_DPS or self.turning(now):
            self.recovery_still_pose = None
        elif (self.recovery_still_pose is None
              or math.dist(position, self.recovery_still_pose[1]) > 0.04
              or self.angle_between(q, self.recovery_still_pose[2]) > 3.0):
            self.recovery_still_pose = (now, position, q)
        elif now - self.recovery_still_pose[0] >= MAP_TURN_SETTLE_SECONDS:
            self.fault = None
            self.anchor = None
            self.loop_closed = False
            self.recovery_still_pose = None
            self.fault_previous = None
            self.previous = (now, position, q, gyro_dps)

    def observe(self, now, position, orientation, gyro_dps=None, loop_closed=False):
        self.note_loop_closure(now, loop_closed)
        norm = math.hypot(*orientation)
        if not all(map(math.isfinite, position)) or not math.isfinite(norm) or norm < 0.9:
            self.recovery_still_pose = None
            if not self.fault:
                self.loop_closed = False
                self.fault_previous = None
            self.fault = "ZED returned an invalid tracking pose."
            return
        q = tuple(value / norm for value in orientation)
        if self.fault:
            self.try_relocalize(now, position, q, gyro_dps)
            return
        previous = self.previous
        if previous is None:
            self.previous = (now, position, q, gyro_dps)
            return
        then, p0, q0, previous_gyro_dps = previous
        elapsed = now - then
        dt = min(max(0.0, elapsed), 0.25)
        distance = math.dist(position, p0)
        angle = self.angle_between(q, q0)
        # Compare to actual time elapsed across a stream gap. A short
        # integration pause is useful even when we cannot diagnose a jump.
        if gyro_dps is None and elapsed > 0:
            self.observe_rotation(now, angle / elapsed)
        jump = self.jumped(elapsed, distance, angle)
        # A large pose correction with little matching physical rotation is
        # especially damaging when tracking settles after a turn.
        if (self.turning(now) and gyro_dps is not None and previous_gyro_dps is not None
                and 0 < elapsed <= 0.12):
            jump |= angle > 12.0 + max(gyro_dps, previous_gyro_dps) * dt
        if jump:
            if self.anchor is None or not self.turning(now):
                self.anchor = previous
            # If this jump is ZED's own loop-closure correction, the scene is
            # already recognized; the pose still has to match a mapped view.
            self.loop_closed = self.recent_loop_closure(now)
            self.fault_previous = (now, position, q)
            self.recovery_still_pose = None
            self.fault = f"ZED tracking jumped {distance:.2f} m / {angle:.0f}° between valid poses."
        else:
            self.previous = (now, position, q, gyro_dps)
            if not self.turning(now):
                self.anchor = None


def gyro_speed(camera, sensors, sl, reference):
    if camera.get_sensors_data(sensors, reference) != sl.ERROR_CODE.SUCCESS:
        return None
    imu = sensors.get_imu_data()
    if not imu.is_available:
        return None
    speed = math.hypot(*imu.get_angular_velocity())
    return speed if math.isfinite(speed) else None


class ZedFusion:
    """SDK fusion with a bounded, quantized preview for the dashboard."""

    def __init__(self, sl, np, camera):
        self.sl, self.np, self.camera = sl, np, camera
        self.cloud = sl.FusedPointCloud()
        self.pending = False
        self.next_request = 0.0

    def start(self, integrate):
        sl = self.sl
        params = sl.SpatialMappingParameters(
            map_type=sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD,
            use_chunk_only=False, max_memory_usage=512,
        )
        params.resolution_meter = VOXEL_SIZE
        status = self.camera.enable_spatial_mapping(params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Could not start ZED Fusion: {status}")
        self.camera.pause_spatial_mapping(not integrate)

    def preview(self, now, integrate):
        if self.pending:
            if self.camera.get_spatial_map_request_status_async() == self.sl.ERROR_CODE.SUCCESS:
                self.pending = False
                if self.camera.retrieve_spatial_map_async(self.cloud) == self.sl.ERROR_CODE.SUCCESS:
                    return self.preview_records(self.cloud.vertices)
        elif integrate and now >= self.next_request:
            self.camera.request_spatial_map_async()
            self.pending = True
            self.next_request = now + 2.0
        return None

    def preview_records(self, vertices):
        np = self.np
        points = np.asarray(vertices, dtype=np.float32).reshape(-1, 4)
        # The SDK retains the full-resolution cloud for export.
        points = points[::max(1, (len(points) + 99_999) // 100_000)]
        xyz = points[:, :3]
        keep = np.all(np.isfinite(xyz) & (np.abs(xyz) < 32000 * VOXEL_SIZE), axis=1)
        points = points[keep]
        data = np.empty(len(points), dtype=[("xyz", "<i2", (3,)), ("rgb", "u1", (3,))])
        data["xyz"] = np.floor(points[:, :3] / VOXEL_SIZE)
        data["rgb"] = np.ascontiguousarray(points[:, 3]).view(np.uint8).reshape(-1, 4)[:, :3]
        return data.tobytes()

    def save(self, path):
        cloud = self.sl.FusedPointCloud()
        result = self.camera.extract_whole_spatial_map(cloud)
        if result != self.sl.ERROR_CODE.SUCCESS:
            raise OSError(f"ZED map extraction returned {result}")
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cloud.save(str(path), self.sl.MESH_FILE_FORMAT.PLY):
            raise OSError("ZED could not write the PLY file")
        return len(cloud.vertices)


class Output:
    def __init__(self):
        # The ZED SDK prints its own log lines to standard output, which would
        # corrupt the binary stream. Keep a private copy of the real stdout for
        # messages and point fd 1 (C and Python prints) at stderr instead.
        stdout = sys.stdout.fileno()
        self.fd = os.dup(stdout)
        sys.stdout.flush()
        os.dup2(sys.stderr.fileno(), stdout)
        sys.stdout = sys.stderr

    def send(self, kind, payload):
        data = pack(kind, payload)
        view = memoryview(data)
        while view:
            written = os.write(self.fd, view)
            view = view[written:]

    def status(self, text, level="info", **extra):
        self.send("status", json.dumps(dict(extra, level=level, text=text)).encode())


def lock_camera():
    import fcntl

    handle = open(LOCK_PATH, "a")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        return None
    return handle


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stream-width", type=int, default=640)
    parser.add_argument("--stream-fps", type=float, default=12.0)
    parser.add_argument("--cloud-width", type=int, default=640, help="Points per row used for mapping")
    parser.add_argument("--map-hz", type=float, default=5.0)
    parser.add_argument("--min-range", type=float, default=0.45,
                        help="Ignore points closer than this to the camera (robot body and folded arms)")
    parser.add_argument("--max-range", type=float, default=4.0)
    parser.add_argument("--save-dir", default=str(Path.home() / "zed-setup" / "maps"))
    parser.add_argument("--bgr-cloud", action="store_true", help="Swap red and blue if map colors look wrong")
    args = parser.parse_args()
    out = Output()
    lock = lock_camera()
    if lock is None:
        out.status("Another camera stream is already using the ZED Mini on the robot.", "fault")
        return 3
    try:
        import cv2
        import numpy as np
        import pyzed.sl as sl
    except ImportError as exc:
        out.status(f"Camera stream needs pyzed, NumPy, and OpenCV: {exc}", "fault")
        return 1

    camera = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.depth_maximum_distance = max(args.max_range, 5.0)
    # The head-mounted ZED Mini is upside down; AUTO uses the IMU to flip the images.
    init.camera_image_flip = sl.FLIP_MODE.AUTO
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        out.status(f"Could not open the ZED Mini: {status}. Run zed_check.py on the robot.", "fault")
        return 1
    tracking = sl.PositionalTrackingParameters()
    tracking.enable_imu_fusion = True
    tracking.enable_area_memory = True
    tracking.set_gravity_as_origin = True
    status = camera.enable_positional_tracking(tracking)
    if status != sl.ERROR_CODE.SUCCESS:
        out.status(f"Could not start positional tracking: {status}.", "fault")
        camera.close()
        return 1

    runtime = sl.RuntimeParameters()
    runtime.measure3D_reference_frame = sl.REFERENCE_FRAME.WORLD
    info = camera.get_camera_information().camera_configuration.resolution
    stream_res = sl.Resolution(args.stream_width, round(args.stream_width * info.height / info.width))
    cloud_res = sl.Resolution(args.cloud_width, round(args.cloud_width * info.height / info.width))
    color_mat, depth_mat, cloud_mat = sl.Mat(), sl.Mat(), sl.Mat()
    pose = sl.Pose()
    sensors = sl.SensorsData()
    vmap = VoxelMap(np)
    integrate = False
    fusion = None
    fusion_running = False
    pose_guard = MappingPoseGuard()
    display_pose = None
    trace = deque(maxlen=18_000)  # about 20 minutes at 15 Hz
    trace_start = time.monotonic()
    credits = {"color": IMAGE_CREDITS, "depth": IMAGE_CREDITS}
    credit_time = {"color": 0.0, "depth": 0.0}
    next_image = next_pose = next_map = 0.0
    frame_interval = 1.0 / args.stream_fps
    failures = 0
    pending = b""
    out.status("ZED Mini open: HD720, NEURAL depth, tracking on.", resolution=[stream_res.width, stream_res.height])
    try:
        while True:
            while select.select([sys.stdin], [], [], 0)[0]:
                data = os.read(sys.stdin.fileno(), 4096)
                if not data:
                    return 0
                lines = (pending + data).split(b"\n")
                pending = lines.pop()[-4096:]
                for line in lines:
                    try:
                        command = json.loads(line)
                    except ValueError:
                        continue
                    op = command.get("op") if isinstance(command, dict) else None
                    if op == "quit":
                        return 0
                    if op == "ack" and command.get("kind") in credits:
                        credits[command["kind"]] = min(IMAGE_CREDITS, credits[command["kind"]] + 1)
                    elif op == "integrate":
                        integrate = command.get("on") is True
                        if fusion and not integrate and fusion_running:
                            camera.pause_spatial_mapping(True)
                            fusion_running = False
                    elif op == "source" or op == "reset":
                        active = "zed" if fusion else "voxel"
                        source = command.get("source") if op == "source" else active
                        if source not in ("zed", "voxel") or (op == "source" and active == source):
                            continue
                        if fusion:
                            camera.disable_spatial_mapping()
                            fusion = None
                            fusion_running = False
                        if source == "zed":
                            try:
                                candidate = ZedFusion(sl, np, camera)
                                candidate.start(False)
                                fusion = candidate
                            except (RuntimeError, AttributeError, TypeError) as exc:
                                out.status(str(exc), "warn", map_source="voxel")
                                if op != "reset":
                                    continue
                        vmap.clear()
                        pose_guard.mapped_views.clear()
                        out.send("voxels", EPOCH.pack(vmap.epoch))
                        if source == "voxel" or fusion:
                            out.status("Map cleared." if op == "reset" else "Map source changed.",
                                       map_source="zed" if fusion else "voxel")
                    elif op == "save":
                        prefix = "zed-fusion" if fusion else "voxel"
                        path = Path(args.save_dir) / time.strftime(f"{prefix}-%Y%m%d-%H%M%S.ply")
                        try:
                            count = fusion.save(path) if fusion else vmap.save_ply(path)
                            out.status(f"Saved {count} points to {path} on the robot.", saved=str(path))
                        except (OSError, RuntimeError) as exc:
                            out.status(f"Could not save the map: {exc}", "warn")
                        else:
                            try:
                                with path.with_suffix(".tracking.jsonl").open("w") as handle:
                                    for sample in trace:
                                        handle.write(json.dumps(sample, allow_nan=False) + "\n")
                            except OSError as exc:
                                out.status(f"Map saved. Could not write the tracking log: {exc}", "warn")
            # Pause before grab where possible. ZED Fusion ingests frames
            # during grab(), before we can inspect their tracking pose.
            before_grab = time.monotonic()
            if integrate and fusion_running:
                pose_guard.observe_rotation(before_grab, gyro_speed(camera, sensors, sl, sl.TIME_REFERENCE.CURRENT))
                if pose_guard.turning(before_grab):
                    camera.pause_spatial_mapping(True)
                    fusion_running = False
            if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                failures += 1
                if fusion and fusion_running:
                    camera.pause_spatial_mapping(True)
                    fusion_running = False
                if failures == 30:
                    out.status("The ZED Mini stopped delivering frames. Check the USB 3 link.", "warn")
                time.sleep(0.01)
                continue
            if failures >= 30:
                out.status("ZED Mini frames restored.")
            failures = 0
            now = time.monotonic()
            state = camera.get_position(pose, sl.REFERENCE_FRAME.WORLD)
            tracking_ok = state == sl.POSITIONAL_TRACKING_STATE.OK
            position = pose.get_translation(sl.Translation()).get().tolist()
            orientation = pose.get_orientation(sl.Orientation()).get().tolist()
            gyro_dps = gyro_speed(camera, sensors, sl, sl.TIME_REFERENCE.IMAGE)
            pose_guard.observe_rotation(now, gyro_dps)
            memory = camera.get_positional_tracking_status().spatial_memory_status
            # LOOP_CLOSED lasts about a frame, so record it even when this
            # frame's tracking state is not OK.
            pose_guard.note_loop_closure(now, memory == sl.SPATIAL_MEMORY_STATUS.LOOP_CLOSED)
            if not tracking_ok:
                pose_guard.recovery_still_pose = None
            if tracking_ok:
                before = pose_guard.fault
                pose_guard.observe(now, position, orientation, gyro_dps)
                if pose_guard.fault and not before:
                    out.status(pose_guard.fault + " Mapping paused. Return slowly to a previously mapped view.", "warn")
                elif before and not pose_guard.fault:
                    out.status("ZED relocalized near a trusted mapped view. Map building can resume.")
            turning = pose_guard.turning(now)
            mapping_ok = integrate and tracking_ok and pose_guard.fault is None and not turning
            if fusion and mapping_ok != fusion_running:
                camera.pause_spatial_mapping(not mapping_ok)
                fusion_running = mapping_ok
            if tracking_ok and not pose_guard.fault and not (integrate and turning):
                display_pose = (position, orientation)
            if now >= next_pose:
                next_pose = now + 1 / 15
                pose_status = {
                    "p": [round(v, 4) if math.isfinite(v) else 0.0 for v in position],
                    "q": [round(v, 5) if math.isfinite(v) else 0.0 for v in orientation],
                    "tracking": getattr(state, "name", str(state).split(".")[-1]),
                    "fps": round(camera.get_current_fps(), 1),
                    "integrating": mapping_ok, "voxels": len(vmap.cells), "epoch": vmap.epoch,
                    "mapping_fault": pose_guard.fault,
                    "map_turn_pause": integrate and turning,
                    "gyro_dps": round(gyro_dps, 1) if gyro_dps is not None else None,
                    "spatial_memory": memory.name,
                    "display_pose": display_pose,
                    "recovery_recognized": pose_guard.loop_closed,
                }
                trace.append(dict(pose_status, elapsed_s=round(now - trace_start, 3),
                                  source="zed" if fusion else "voxel"))
                out.send("pose", json.dumps(pose_status).encode())
            for kind in credits:
                if credits[kind] == 0 and now - credit_time[kind] > CREDIT_TIMEOUT:
                    credits[kind] = 1
            if now >= next_image:
                next_image = now + frame_interval
                if credits["color"]:
                    camera.retrieve_image(color_mat, sl.VIEW.LEFT, sl.MEM.CPU, stream_res)
                    ok, jpeg = cv2.imencode(".jpg", np.ascontiguousarray(color_mat.get_data()[:, :, :3]),
                                            [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if ok:
                        out.send("color", jpeg.tobytes())
                        credits["color"] -= 1
                        credit_time["color"] = now
                if credits["depth"]:
                    camera.retrieve_measure(depth_mat, sl.MEASURE.DEPTH, sl.MEM.CPU, stream_res)
                    depth = depth_mat.get_data()
                    valid = np.isfinite(depth)
                    scaled = np.zeros(depth.shape, dtype=np.uint8)
                    scaled[valid] = np.clip((depth[valid] - 0.3) / (args.max_range - 0.3) * 255, 0, 255)
                    image = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_TURBO)
                    image[~valid] = 0
                    ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if ok:
                        out.send("depth", jpeg.tobytes())
                        credits["depth"] -= 1
                        credit_time["depth"] = now
            if fusion:
                if mapping_ok and now >= fusion.next_request:
                    pose_guard.remember_mapped_view()
                records = fusion.preview(now, mapping_ok)
                if records is not None:
                    vmap.epoch += 1
                    out.send("voxels", EPOCH.pack(vmap.epoch) + records)
            elif mapping_ok and now >= next_map:
                next_map = now + 1.0 / args.map_hz
                pose_guard.remember_mapped_view()
                camera.retrieve_measure(cloud_mat, sl.MEASURE.XYZRGBA, sl.MEM.CPU, cloud_res)
                cloud = cloud_mat.get_data().reshape(-1, 4)
                rgba = np.ascontiguousarray(cloud[:, 3]).view(np.uint8).reshape(-1, 4)
                rgb = rgba[:, [2, 1, 0]] if args.bgr_cloud else rgba[:, :3]
                records = vmap.add(np.ascontiguousarray(cloud[:, :3]), rgb, position, args.min_range, args.max_range)
                if records:
                    out.send("voxels", EPOCH.pack(vmap.epoch) + records)
    except (BrokenPipeError, KeyboardInterrupt):
        return 0
    finally:
        if fusion:
            camera.disable_spatial_mapping()
        camera.disable_positional_tracking()
        camera.close()


if __name__ == "__main__":
    sys.exit(main())
