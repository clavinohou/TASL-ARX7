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
import json
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
VOXEL_SIZE = 0.05
MAX_PAYLOAD = 8 << 20
KINDS = {"color": ord("C"), "depth": ord("D"), "pose": ord("P"), "voxels": ord("V"), "status": ord("S")}
NAMES = {code: name for name, code in KINDS.items()}
IMAGE_CREDITS = 2      # images in flight per stream, so video never queues ahead of control traffic
CREDIT_TIMEOUT = 3.0
CONFIRM_FRAMES = 2     # a voxel must be seen in this many frames before it is sent
REFINE_FRAMES = (8, 32)  # resend the averaged color at these counts
LOCK_PATH = "/dev/shm/arx-zed.lock"


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


class Output:
    def __init__(self):
        self.fd = sys.stdout.fileno()

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
    parser.add_argument("--cloud-width", type=int, default=320, help="Points per row used for mapping")
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
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        out.status(f"Could not open the ZED Mini: {status}. Run zed_check.py on the robot.", "fault")
        return 1
    tracking = sl.PositionalTrackingParameters()
    tracking.enable_imu_fusion = True
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
    vmap = VoxelMap(np)
    integrate = False
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
                    elif op == "reset":
                        vmap.clear()
                        out.send("voxels", EPOCH.pack(vmap.epoch))
                        out.status("Map cleared.")
                    elif op == "save":
                        path = Path(args.save_dir) / time.strftime("map-%Y%m%d-%H%M%S.ply")
                        try:
                            count = vmap.save_ply(path)
                            out.status(f"Saved {count} voxels to {path} on the robot.", saved=str(path))
                        except OSError as exc:
                            out.status(f"Could not save the map: {exc}", "warn")
            if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                failures += 1
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
            if now >= next_pose:
                next_pose = now + 1 / 15
                orientation = pose.get_orientation(sl.Orientation()).get().tolist()
                out.send("pose", json.dumps({
                    "p": [round(v, 4) for v in position], "q": [round(v, 5) for v in orientation],
                    "tracking": getattr(state, "name", str(state).split(".")[-1]), "fps": round(camera.get_current_fps(), 1),
                    "integrating": integrate and tracking_ok, "voxels": len(vmap.cells), "epoch": vmap.epoch,
                }).encode())
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
            if integrate and tracking_ok and now >= next_map:
                next_map = now + 1.0 / args.map_hz
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
        camera.disable_positional_tracking()
        camera.close()


if __name__ == "__main__":
    sys.exit(main())
