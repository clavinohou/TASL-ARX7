"""
VR Dual Arm Controller

This application controls dual robotic arms using VR controller input.
It now receives VR data through the fan-out service on port 7002.

To use:
1. Start the fan-out service: python vr_fanout.py
2. Start this application: python vr_dual_arm.py
"""

from bimanual import SingleArm
import socket
import struct
from typing import Dict, Any
from scipy.spatial.transform import Rotation as R
import numpy as np

# —– Arm configurations (set your real CAN ports here)
left_arm_config: Dict[str, Any] = {
    "can_port": "can0",
    "type": 0,
}
right_arm_config: Dict[str, Any] = {
    "can_port": "can1",
    "type": 1,
}

left_arm  = SingleArm(left_arm_config)
right_arm = SingleArm(right_arm_config)

# Button masks
PRIMARY    = (1 << 0)
SECONDARY  = (1 << 1)
TRIGGERBTN = (1 << 2)
GRIPBTN    = (1 << 3)
MENUBTN    = (1 << 4)

# UDP (receives from fan-out service)
UDP_IP   = "127.0.0.1"  # Listen on localhost for fan-out data
UDP_PORT = 7002         # Fan-out port for vr_dual_arm

# Packet format: time + [left ctrl] + [right ctrl]
FMT        = "<d3f4f2f2fB3f4f2f2fB"
PACKET_SIZE = struct.calcsize(FMT)

# VR→arm orientation offset (same for both)
VR2ARM = R.from_quat([0.5, 0.5, -0.5, -0.5])

def parse_packet(data: bytes):
    if len(data) < PACKET_SIZE:
        raise ValueError(f"Packet too short: {len(data)} < {PACKET_SIZE}")
    up = struct.unpack(FMT, data[:PACKET_SIZE])
    time = up[0]
    # left ctrl
    l0 = 1
    left = {
      "pos":     up[l0+0:l0+3],
      "rot":     up[l0+3:l0+7],
      "joy":     up[l0+7:l0+9],
      "trigger": up[l0+9],
      "grip":    up[l0+10],
      "buttons": up[l0+11],
    }
    # right ctrl
    r0 = 1 + 12
    right = {
      "pos":     up[r0+0:r0+3],
      "rot":     up[r0+3:r0+7],
      "joy":     up[r0+7:r0+9],
      "trigger": up[r0+9],
      "grip":    up[r0+10],
      "buttons": up[r0+11],
    }
    return time, left, right

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening on {UDP_IP}:{UDP_PORT} (fan-out service, expecting {PACKET_SIZE}B packets)")

    # State for left grip
    left_grip_refed  = False
    left_pos_ref     = np.zeros(3)
    left_quat_ref    = R.identity()
    vr_left_pos_ref  = np.zeros(3)
    vr_left_quat_ref = R.identity()

    # State for right grip
    right_grip_refed  = False
    right_pos_ref     = np.zeros(3)
    right_quat_ref    = R.identity()
    vr_right_pos_ref  = np.zeros(3)
    vr_right_quat_ref = R.identity()

    try:
        while True:
            data, _ = sock.recvfrom(2048)
            try:
                t, left_ctrl, right_ctrl = parse_packet(data)
            except ValueError as e:
                print("  ❗", e)
                continue

            # -- BUTTONS: Homes --
            if   left_ctrl["buttons"] & PRIMARY:
                left_arm.go_home()
            if   right_ctrl["buttons"] & PRIMARY:
                right_arm.go_home()

            # -- LEFT ARM GRIP & MOVE --
            if not (left_ctrl["buttons"] & GRIPBTN):
                left_grip_refed = False
            else:
                if not left_grip_refed:
                    # first frame grip pressed → grab reference
                    left_grip_refed = True
                    # get current arm pose (pos + quat)
                    ee = left_arm.get_ee_pose()  # [x,y,z, w,x,y,z]
                    left_pos_ref = np.array(ee[:3])
                    w,x,y,z    = ee[3:]
                    left_quat_ref = R.from_quat([x,y,z,w])
                    # get VR ref
                    vr_left_pos_ref  = np.array(left_ctrl["pos"])
                    vr_left_quat_ref = R.from_quat(left_ctrl["rot"])

                # compute delta pos
                curr_vr_pos = np.array(left_ctrl["pos"])
                dp = curr_vr_pos - vr_left_pos_ref
                # simple axes remap: (z, -x, y)
                target_pos = left_pos_ref + np.array([dp[2], -dp[0], dp[1]])

                # compute delta orientation
                curr_vr_quat = R.from_quat(left_ctrl["rot"])
                vr_rel       = curr_vr_quat * vr_left_quat_ref.inv()
                arm_rel      = VR2ARM * vr_rel * VR2ARM.inv()
                targ_quat    = left_quat_ref * arm_rel
                x,y,z,w      = targ_quat.as_quat()
                target_quat  = [w,x,y,z]

                # send
                left_arm.set_ee_pose(pos=target_pos, quat=target_quat)
                # optional: use trigger for grip strength
                left_arm.set_catch_pos(5 - left_ctrl["trigger"] * 5)

            # -- RIGHT ARM GRIP & MOVE --
            if not (right_ctrl["buttons"] & GRIPBTN):
                right_grip_refed = False
            else:
                if not right_grip_refed:
                    right_grip_refed = True
                    ee = right_arm.get_ee_pose()
                    right_pos_ref = np.array(ee[:3])
                    w,x,y,z      = ee[3:]
                    right_quat_ref = R.from_quat([x,y,z,w])
                    vr_right_pos_ref  = np.array(right_ctrl["pos"])
                    vr_right_quat_ref = R.from_quat(right_ctrl["rot"])

                dp = np.array(right_ctrl["pos"]) - vr_right_pos_ref
                target_pos = right_pos_ref + np.array([dp[2], -dp[0], dp[1]])

                curr_vr_quat = R.from_quat(right_ctrl["rot"])
                vr_rel       = curr_vr_quat * vr_right_quat_ref.inv()
                arm_rel      = VR2ARM * vr_rel * VR2ARM.inv()
                targ_quat    = right_quat_ref * arm_rel
                x,y,z,w      = targ_quat.as_quat()
                target_quat  = [w,x,y,z]

                right_arm.set_ee_pose(pos=target_pos, quat=target_quat)
                right_arm.set_catch_pos(5 - right_ctrl["trigger"] * 5)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
