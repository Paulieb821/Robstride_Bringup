#!/usr/bin/env python3
# oculus_velocity_plot.py
"""
Record Quest controller vel for a few seconds, then plot:
 • world-frame vel (X,Y,Z)
 • robot-frame vel (X,Y,Z)
Press Side-Grip to calibrate origin (optional).
Press B to stop recording and show plots.
"""

import time, sys, signal
import numpy as np
import openvr
import triad_openvr as tov
import matplotlib.pyplot as plt

# ────────── init OpenVR ──────────
openvr.init(openvr.VRApplication_Other)
vr      = openvr.VRSystem()
tov_api = tov.triad_openvr()

RIGHT = openvr.TrackedControllerRole_RightHand
ctrl_idx = vr.getTrackedDeviceIndexForControllerRole(RIGHT)
if ctrl_idx == openvr.k_unTrackedDeviceIndexInvalid:
    print("No right controller → exit"); sys.exit(1)
ctrl = tov_api.devices[f"controller_{ctrl_idx}"]

# ────────── calibration ──────────
print("Hold controller still & press side-GRIP to calibrate origin (or wait 3 s)…")
origin = None
start = time.time()
while origin is None and time.time()-start < 3:
    pos = np.array(ctrl.get_pose_quaternion()[:3])
    _, st = vr.getControllerState(ctrl_idx)
    if st.ulButtonPressed & (1<<openvr.k_EButton_Grip):
        origin = pos.copy()
        print("Calibrated.")
    time.sleep(0.02)
if origin is None:
    origin = pos  # fallback
print("Recording… Move along X, Y, Z. Press B to stop.")

# catch Ctrl-C
signal.signal(signal.SIGINT, lambda s,f: sys.exit(0))

# ────────── collect data ──────────
t0 = time.time()
times       = []
world_vels  = []  # [ [vx,vy,vz], ... ]
robot_vels  = []
while True:
    now = time.time()
    dt  = now - t0
    times.append(dt)

    # read raw vel
    raw = ctrl.get_velocity()
    wv  = np.array([raw[0], raw[1], raw[2]])
    world_vels.append(wv)

    # map → robot frame (+X, +Y, +Z)robot = (+X, -Z, +Y)world
    rv = np.array([ wv[0], -wv[2], wv[1] ])
    robot_vels.append(rv)

    # check for B button to stop
    _, st = vr.getControllerState(ctrl_idx)
    if st.ulButtonPressed & (1<<openvr.k_EButton_ApplicationMenu):
        break

    # cap at 15 s max
    if dt > 15:
        break

    time.sleep(1/60)  # sample ~60 Hz

world_vels = np.vstack(world_vels)
robot_vels = np.vstack(robot_vels)
times      = np.array(times)

# ────────── plotting ──────────
# 1) world frame
plt.figure()
plt.plot(times, world_vels[:,0], label='X_w')
plt.plot(times, world_vels[:,1], label='Y_w')
plt.plot(times, world_vels[:,2], label='Z_w')
plt.title("Controller Velocity in SteamVR World Frame")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.legend()
plt.grid(True)

# 2) robot frame
plt.figure()
plt.plot(times, robot_vels[:,0], label='X_r')
plt.plot(times, robot_vels[:,1], label='Y_r')
plt.plot(times, robot_vels[:,2], label='Z_r')
plt.title("Controller Velocity in Robot Frame")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.legend()
plt.grid(True)

plt.show()
