"""Read-only X7s body telemetry. Prints one JSON snapshot per line on Linux."""

from collections import Counter
import json
import socket
import struct
import time


BODY = {1: "Lift", 4: "Waist pivot", 5: "Head yaw", 6: "Head pitch"}
FRAME = struct.Struct("=IB3x8s")
FILTER = struct.Struct("=II")
IDS = (0, *BODY, 0x701, 0x702, 0x721, 0x725)
EXPIRES = 0.5
SCALE = 10 / 32768


class Feedback:
    def __init__(self):
        self.wheels = None
        self.wheel_raw = None
        self.peak = [0.0] * 4
        self.drive = None
        self.last_drive = None
        self.motors = {}
        self.motor_commands = {}
        self.switch = None
        self.count = Counter()

    def frame(self, ident, data, local, now):
        # These layouts match the ChassisType1 and MotorType3 decoders in the
        # installed X7s SDK. Body positions are raw motor angles before unwrapping.
        if ident == 0x702 and not local:
            speed = [value * SCALE for value in struct.unpack(">4h", data)]
            self.wheels = (now, speed)
            self.wheel_raw = (now, data.hex(" "))
            self.peak = [max(previous, abs(value)) for previous, value in zip(self.peak, speed)]
            self.count["wheel_reports"] += 1
        elif ident == 0x701 and local:
            x, yaw, mode, y = struct.unpack(">4h", data)
            command = {"mode": mode, "x": x * 2 / 660, "y": y * -2 / 660, "yaw": yaw * -4 / 660}
            self.drive = (now, command)
            if mode == 1 and (x or yaw or y):
                self.last_drive = command
                self.count["nonzero_drive"] += 1
            self.count["chassis_commands"] += 1
        elif ident == 0 and not local and data[0] & 0x0f in BODY:
            motor_id = data[0] & 0x0f
            pos = int.from_bytes(data[1:3], "big")
            vel = (data[3] << 4) | (data[4] >> 4)
            self.motors[motor_id] = (now, {
                "angle": pos * 25 / 65535 - 12.5,
                "speed": vel * 90 / 4095 - 45,
                "status": data[0] >> 4,
            })
        elif ident in BODY and local:
            kind = "Control"
            if data[:7] == b"\xff" * 7:
                kind = {0xfc: "Enable", 0xfd: "Disable", 0xfe: "Set zero",
                        0xfb: "Clear error"}.get(data[7], "Special command")
                self.count[f"{kind} {BODY[ident]}"] += 1
            self.motor_commands[ident] = (now, kind)
        elif ident in (0x721, 0x725) and not local:
            self.switch = (now, f"0x{ident:03x}  {data.hex(' ')}")

    def snapshot(self, now):
        def recent(value):
            return value[1] if value and now - value[0] < EXPIRES else None

        return {
            "wheels": recent(self.wheels), "wheel_raw": recent(self.wheel_raw), "peak": self.peak, "drive": recent(self.drive),
            "last_drive": self.last_drive, "switch": recent(self.switch), "count": self.count,
            "motors": {label: {"id": ident, "feedback": recent(self.motors.get(ident)),
                               "command": recent(self.motor_commands.get(ident))}
                       for ident, label in BODY.items()},
        }


def main():
    state = Feedback()
    next_report = time.monotonic()
    try:
        with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
            mask = socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG | socket.CAN_SFF_MASK
            bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                           b"".join(FILTER.pack(ident, mask) for ident in IDS))
            bus.bind(("can5",))
            bus.settimeout(0.1)
            while True:
                try:
                    frame, _, flags, _ = bus.recvmsg(FRAME.size)
                    ident, length, data = FRAME.unpack(frame)
                    if length == 8:
                        state.frame(ident, data, bool(flags & socket.MSG_DONTROUTE), time.monotonic())
                except socket.timeout:
                    pass
                now = time.monotonic()
                if now >= next_report:
                    print(json.dumps(state.snapshot(now)), flush=True)
                    next_report = now + 0.1
    except (BrokenPipeError, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()
