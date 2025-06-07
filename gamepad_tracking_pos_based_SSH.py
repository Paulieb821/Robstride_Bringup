#!/usr/bin/env python3
"""
Run via SSH with:
sudo /home/lidar/anaconda3/envs/kscale/bin/python \
    /home/lidar/Robstride_Bringup/gamepad_tracking_pos_based_SSH.py
"""

import threading
import time

import mujoco as mj
import can
import robstride
import numpy as np
import serial
import sys
from inputs import get_gamepad

from servocontrol import GripperController
from robot_control_utils.NRIK import NRIK

########################
# CONFIGURATION SECTION
########################

urdf_path = 'robot_models/Sim_Arm_4DOF_May_25/robot.xml'
site_name = 'endeff'
command_rate = 20
speed = 0.2
max_velocity = speed / command_rate
acceleration = 0.2 * max_velocity  # Smoothing

motor_ids    = [1, 2, 3, 4]
motor_ratios = [1, 1, -1, -3]

reachable_sphere_center = np.array([0, 0, 0.115])
reachable_sphere_radius = 0.9

using_gripper = True

########################
# GAMEPAD LISTENER
########################

# Shared state updated by listener thread
gamepad_state = {
    'x':      0,    # raw ABS_X
    'y':      0,    # raw ABS_Y
    'hat_y':  0,    # raw ABS_HAT0Y (-1/0/1)
    'open':   0,    # BTN_SOUTH (A)
    'close':  0,    # BTN_EAST  (B)
    'toggle': 0,    # BTN_NORTH (X)
}

def _gamepad_listener():
    DEADZONE = 0.2
    while True:
        events = get_gamepad()
        for e in events:
            print(f"[GAMEPAD] ev_type={e.ev_type} code={e.code} state={e.state}")
            # Analog
            if e.ev_type == 'Absolute':                
                if e.code == 'ABS_X':
                    gamepad_state['x'] = e.state
                elif e.code == 'ABS_Y':
                    gamepad_state['y'] = e.state
                elif e.code == 'ABS_HAT0Y':
                    gamepad_state['hat_y'] = e.state
            # Buttons
            elif e.ev_type == 'Key':
                if e.code == 'BTN_SOUTH':  # A
                    gamepad_state['open'] = e.state
                elif e.code == 'BTN_EAST':  # B
                    gamepad_state['close'] = e.state
                elif e.code == 'BTN_NORTH': # X
                    gamepad_state['toggle'] = e.state
        # rate limit
        time.sleep(0.005)

# start in background
threading.Thread(target=_gamepad_listener, daemon=True).start()

########################
# INITIALIZATION
########################

# MuJoCo & IK
model = mj.MjModel.from_xml_path(urdf_path)
data  = mj.MjData(model)
site  = data.site(site_name)
ik    = NRIK(model, data, site)
mj.mj_forward(model, data)

endeff_pos = site.xpos.copy()
joint_pos  = ik.solveIK_3dof(endeff_pos)
joint_limits = model.jnt_range[:model.njnt]
velocity = np.zeros(3)


# Correct joystick range values for 16-bit input
X_MIN, X_MAX = -32768, 32767
Y_MIN, Y_MAX = -32768, 32767

x_center    = 128  # Joystick is centered at 0
x_halfrange = (X_MAX - X_MIN) / 2  # Half the range for normalization  

y_center    = -129  # Joystick is centered at 0
y_halfrange = (Y_MAX - Y_MIN) / 2  # Half the range for normalization    

# Gripper init
if using_gripper:
    try:
        gripper = GripperController(
        port="/dev/ttyUSB0",
        baud=9600,
        servo_ids=(8, 11),
        pulse_open=[500, 2500],
        pulse_closed=[2500, 500],
        step_us=50
    )
    except Exception as e:
        print(f"[WARNING] Gripper init failed: {e}")
        using_gripper = False

########################
# MAIN CONTROL LOOP
########################

# ──────────────────── CAN ­HELPER (NEW) ────────────────────
def send_hold(client):
    """Keep drives alive with last valid joint_pos."""
    for i, mid in enumerate(motor_ids):
        client.write_param(mid, 'loc_ref', joint_pos[i] * motor_ratios[i])

        
with can.Bus(interface='socketcan', channel='can0', bitrate=1_000_000) as bus:
    rs_client = robstride.Client(bus)

    # sanity-check zeroing
    for mid in motor_ids:
        if rs_client.read_param(mid, 'mechpos') > 2:
            print("Zeroing error: re-zero hardware.")
            sys.exit(1)

    # enable motors
    for mid in motor_ids:
        rs_client.write_param(mid, 'run_mode', robstride.RunMode.Position)
        rs_client.enable(mid)

    while True:
        # --- GRIPPER BUTTONS ---
    
        if using_gripper and gamepad_state['open'] == 1:
            gamepad_state['open'] = 0
            gripper.open_step()

        if using_gripper and gamepad_state['close'] == 1:
            gamepad_state['close'] = 0
            gripper.close_step()

        if using_gripper and gamepad_state['toggle'] == 1:
            gamepad_state['toggle'] = 0
            gripper.toggle()

        # # --- TRANSLATIONAL INPUT ---
        # print("This is the game pad state x : ", gamepad_state['x'])
        # print("This is the game pad state y : ", gamepad_state['y'])
         # print("This is the game pad state x normalized : ", x_norm)
        # print("This is the game pad state y normalized : ", y_norm)

        x_norm = (gamepad_state['x'] - x_center) / x_halfrange
        y_norm = (gamepad_state['y'] - y_center) / y_halfrange
        x_norm = max(-1.0, min(1.0, x_norm))
        y_norm = max(-1.0, min(1.0, y_norm))

        # deadzone threshold
        thr = 0.15
        direction = np.zeros(3)
        if abs(x_norm) > thr:
            direction[0] = x_norm
        if abs(y_norm) > thr:
            direction[1] = y_norm

        # d‑pad for Z
        hat = gamepad_state['hat_y']
        if hat != 0:
            direction[2] = -hat

        # smoothing & reach checks
        if np.linalg.norm(direction) > 0:
            direction = direction / np.linalg.norm(direction)
            target_velocity = direction * max_velocity
        else:
            target_velocity = np.zeros(3)

        delta_v = target_velocity - velocity
        delta_v = np.clip(delta_v, -acceleration, acceleration)
        velocity += delta_v

        proposed_pos = endeff_pos + velocity
        if np.linalg.norm(proposed_pos - reachable_sphere_center) > reachable_sphere_radius:
            print("Blocked: outside reachable zone.")
            time.sleep(1/command_rate)
            continue

        proposed_joints = ik.solveIK_3dof(proposed_pos)
        if any((proposed_joints[i] < joint_limits[i][0]) or
               (proposed_joints[i] > joint_limits[i][1])
               for i in range(len(motor_ids))):
            print("Blocked: joint limit reached.")
            time.sleep(1/command_rate)
            continue

        # accept and send
        endeff_pos = proposed_pos
        joint_pos  = proposed_joints
        # print("EE:", np.round(endeff_pos,2), "J:", np.round(joint_pos,2))

        for i, mid in enumerate(motor_ids):
            rs_client.write_param(mid, 'loc_ref', joint_pos[i] * motor_ratios[i])

        time.sleep(1/command_rate)
