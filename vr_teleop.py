#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading, time, sys
import numpy as np
import triad_openvr as tov                # NEW
import openvr                             #   "
import mujoco as mj, can, robstride, serial
from servocontrol import GripperController
from robot_control_utils.NRIK import NRIK

########################
# CONFIGURATION SECTION
########################
urdf_path            = 'robot_models/Sim_Arm_4DOF_May_25/robot.xml'
site_name            = 'endeff'
command_rate         = 20                 # Hz
speed                = 0.2
max_velocity         = speed / command_rate
acceleration         = 0.2 * max_velocity
motor_ids            = [1, 2, 3, 4]
motor_ratios         = [1, 1, -1, -3]
reachable_center     = np.array([0, 0, 0.115])
reachable_radius     = 0.9
using_gripper        = False
right_role           = openvr.TrackedControllerRole_RightHand

#####################################
# 1  OpenVR initialisation & helpers
#####################################
openvr.init(openvr.VRApplication_Other)
vr_sys   = openvr.VRSystem()
v        = tov.triad_openvr()
right_id = vr_sys.getTrackedDeviceIndexForControllerRole(right_role)
if right_id == openvr.k_unTrackedDeviceIndexInvalid:
    print("No right-hand controller found — aborting.")
    sys.exit(1)
ctrl      = v.devices[f"controller_{right_id}"]

def get_controller_state():
    """Return (pos[3], buttons dict) for the right controller."""
    pose = ctrl.get_pose_quaternion()
    pos  = np.array([pose[0], pose[1], pose[2]])
    _, state = vr_sys.getControllerState(right_id)
    pressed  = state.ulButtonPressed
    return pos, {
        'grip'  : bool(pressed & (1 << openvr.k_EButton_Grip)),
        'a'     : bool(pressed & (1 << openvr.k_EButton_A)),
        'b'     : bool(pressed & (1 << openvr.k_EButton_ApplicationMenu)),
        'trigger': bool(pressed & (1 << openvr.k_EButton_SteamVR_Trigger)),
        # add others if needed
    }

#####################################
# 2  Calibration — press Grip once
#####################################
print(">>> Hold right controller at current EE pose and press the side-grip to calibrate …")
while True:
    p, btn = get_controller_state()
    if btn['grip']:
        origin_ctrl_pos = p.copy()
        break
    time.sleep(0.01)

########################
# INITIALISATION 
########################
model   = mj.MjModel.from_xml_path(urdf_path)
data    = mj.MjData(model)
site    = data.site(site_name)
ik      = NRIK(model, data, site)
mj.mj_forward(model, data)

endeff_pos   = site.xpos.copy()
joint_pos    = ik.solveIK_3dof(endeff_pos)
joint_limits = model.jnt_range[:model.njnt]
velocity     = np.zeros(3)

# Initialise gripper
if using_gripper:
    try:
        gripper = GripperController(port="/dev/ttyUSB0", baud=9600,
                                    servo_ids=(8, 11),
                                    pulse_open=[500, 2500],
                                    pulse_closed=[2500, 500],
                                    step_us=50)
    except Exception as e:
        print(f"[WARNING] Gripper init failed: {e}")
        using_gripper = False

#################################################
# MAIN CONTROL LOOP 
#################################################
def send_hold(client):
    for i, mid in enumerate(motor_ids):
        client.write_param(mid, 'loc_ref', joint_pos[i] * motor_ratios[i])

with can.Bus(interface='socketcan', channel='can0', bitrate=1_000_000) as bus:
    rs_client = robstride.Client(bus)

    # zero-check, enable motors 
    for mid in motor_ids:
        if rs_client.read_param(mid, 'mechpos') > 2:
            print("Zeroing error: re-zero hardware."); sys.exit(1)
        rs_client.write_param(mid, 'run_mode', robstride.RunMode.Position)
        rs_client.enable(mid)

    while True:
        # ---------- Read controller pose & buttons ----------
        ctrl_pos, buttons = get_controller_state()
        delta = ctrl_pos - origin_ctrl_pos      # metres
        print(f"Original position {origin_ctrl_pos},\ndelta: {delta}")
        direction = np.array([delta[0], -delta[2], delta[1]])  # rx = vr x, ry = vr y, rz = vr z
        print(f"Axis adjusted position : {direction}")
        direction *= 0.5 # apply some scaling to the joystick input

        # buttons → gripper 
        if using_gripper:
            if buttons['trigger']:
                gripper.toggle()
            elif buttons['b']:
                gripper.close()
            elif buttons['a']:
                gripper.open()

        # ---------- Cartesian velocity limiting ----------
        if np.linalg.norm(direction) > 0.01:
            direction /= np.linalg.norm(direction)
            target_velocity = direction * max_velocity
        else:
            target_velocity = np.zeros(3)

        dv = np.clip(target_velocity - velocity, -acceleration, acceleration)
        velocity += dv
        proposed_pos = endeff_pos + velocity

        # reachable sphere check
        if np.linalg.norm(proposed_pos - reachable_center) > reachable_radius:
            print("Blocked: outside reachable zone."); send_hold(rs_client); continue

        # IK & joint-limit check
        proposed_joints = ik.solveIK_3dof(proposed_pos)
        if any((proposed_joints[i] < joint_limits[i][0]) or
               (proposed_joints[i] > joint_limits[i][1])
               for i in range(len(motor_ids))):
            print("Blocked: joint limit reached."); send_hold(rs_client); continue

        # ---------- Accept & send ----------
        endeff_pos = proposed_pos; joint_pos = proposed_joints
        for i, mid in enumerate(motor_ids):
            rs_client.write_param(mid, 'loc_ref', joint_pos[i] * motor_ratios[i])
        time.sleep(1/command_rate)
