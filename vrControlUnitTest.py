#!/usr/bin/env python3
# oculus_controller_test.py
"""
Quick-and-dirty sanity-check for Quest / SteamVR controller input
• prints position (m), quat (w,x,y,z), velocity (m/s) every 50 ms
• dumps pressed buttons as ASCII list
• press the side-Grip once to re-zero (calibrate) the origin
Stop with Ctrl-C.
"""

import time, sys, signal
import numpy as np
import openvr               
import triad_openvr as tov  

# ────────── util helpers ──────────
def euler_from_quat(q):
    """Convert (w, x, y, z) quaternion to yaw-pitch-roll in degrees."""
    w, x, y, z = q
    ysqr = y * y
    t0 = +2.0 * (w * z + x * y)
    t1 = +1.0 - 2.0 * (ysqr + z * z)
    yaw = np.degrees(np.arctan2(t0, t1))
    t2 = +2.0 * (w * x - y * z)
    t2 = np.clip(t2, -1.0, +1.0)
    pitch = np.degrees(np.arcsin(t2))
    t3 = +2.0 * (w * y + z * x)
    t4 = +1.0 - 2.0 * (x * x + ysqr)
    roll = np.degrees(np.arctan2(t3, t4))
    return yaw, pitch, roll

def human_buttons(mask):
    """Return list of pressed button names from bit-mask."""
    mapping = {
        openvr.k_EButton_Grip          : "GRIP-SIDE",
        openvr.k_EButton_A             : "A",
        openvr.k_EButton_ApplicationMenu: "B",
        openvr.k_EButton_System        : "SYS",
        openvr.k_EButton_SteamVR_Touchpad: "TOUCHPAD",
        openvr.k_EButton_SteamVR_Trigger : "TRIGGER-FINGER",
    }
    return [name for eid, name in mapping.items() if mask & (1 << eid)]

# ────────── init OpenVR ──────────
openvr.init(openvr.VRApplication_Other)
vr  = openvr.VRSystem()
tov_api = tov.triad_openvr()

RIGHT = openvr.TrackedControllerRole_RightHand
LEFT  = openvr.TrackedControllerRole_LeftHand

ids = {role : vr.getTrackedDeviceIndexForControllerRole(role) for role in (LEFT, RIGHT)}
for role, idx in ids.items():
    if idx == openvr.k_unTrackedDeviceIndexInvalid:
        print(f"local [WARN] No {'RIGHT' if role==RIGHT else 'LEFT'} controller detected!")

# pick one controller (right) for calibration demo
ctrl_idx = ids[RIGHT]
if ctrl_idx == openvr.k_unTrackedDeviceIndexInvalid:
    print("local No right controller → abort"); sys.exit(1)
ctrl = tov_api.devices[f"controller_{ctrl_idx}"]

# ────────── calibration (press GRIP once) ──────────
print(">>> local Hold right controller at desired origin and squeeze the side-GRIP to calibrate")
origin = None
while origin is None:
    pos = np.array([ctrl.get_pose_quaternion()[:3]])
    _, st = vr.getControllerState(ctrl_idx)
    if st.ulButtonPressed & (1 << openvr.k_EButton_Grip):
        origin = pos
        print("local ✔ calibrated at", np.round(origin, 3))
    time.sleep(0.02)

# graceful Ctrl-C
signal.signal(signal.SIGINT, lambda s,f: sys.exit(0))

# ────────── main loop ──────────
print("\n local Streaming … (Ctrl-C to stop)\n")
while True:
    pose = ctrl.get_pose_quaternion()

    # position / orientation
    pos = np.array([pose[0], pose[1], pose[2]]) - origin
    quat = (pose[3], pose[4], pose[5], pose[6])
    yaw, pitch, roll = euler_from_quat(quat)

    # velocity (SteamVR already predicts ~11 ms ahead)
    vel = np.array([ctrl.get_velocity()[:3]])  # m/s

    # buttons & battery
    _, state = vr.getControllerState(ctrl_idx)
    pressed = human_buttons(state.ulButtonPressed)

    # print a tidy one-liner
    print(f"pos {pos.round(3)}  yaw/pitch/roll {yaw:.1f}/{pitch:.1f}/{roll:.1f}°  "
          f"vel {vel.round(2)}  buttons {pressed or '-'}  ",
          end='\r', flush=True)

    time.sleep(0.05)   # 20 Hz
