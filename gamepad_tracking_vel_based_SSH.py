"""
use this command when on ssh : 
sudo /home/lidar/anaconda3/envs/kscale/bin/python /home/lidar/Robstride_Bringup/keyboard_tracking_pos_based_SSH.py
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

# Mujoco configuration
urdf_path = 'robot_models/Sim_Arm_4DOF_May_25/robot.xml'
site_name = 'endeff'

# Control settings
command_rate = 10   # Hz
max_speed = 0.2     # Speed of end effector m/s
accel = 1.0         # Acceleration on start/stop to avoid vibration
null_speed = 0.2    # Speed at which J1 moves through the null space rad/s

# Motor Setup
motor_ratios = np.array([1, 1, -1, -3]) 
motor_ids = [1, 2, 3, 4]

# Reachability
reachable_sphere_center = np.array([0, 0, 0.115])
reachable_sphere_radius = 0.9       

# Hybrid hold tuning
Kp_hold = 15.0
max_hold_speed = 2

# Gripper
using_gripper = True

######################
# INITIALIZATION
######################

# Load MuJoCo model
model = mj.MjModel.from_xml_path(urdf_path)
data = mj.MjData(model)
site = data.site(site_name)
jac_pos = np.zeros((3, model.nv))
jac_rot = np.zeros((3, model.nv))

# Extract joint limits
joint_limits = model.jnt_range[:model.njnt]  # (njnt, 2)
# Correct joystick range values for 16-bit input

X_MIN, X_MAX = -32768, 32767
Y_MIN, Y_MAX = -32768, 32767

x_center    = 128  # Joystick is centered at 0
x_halfrange = (X_MAX - X_MIN) / 2  # Half the range for normalization  

y_center    = -129  # Joystick is centered at 0
y_halfrange = (Y_MAX - Y_MIN) / 2  # Half the range for normalization    


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


# Initialize gripper if needed
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


with can.Bus(interface='socketcan', channel='can0', bitrate=1000000) as bus:
    rs_client = robstride.Client(bus)

    # Check for issue where zeroing leads to values in the 6 range
    for id in motor_ids:
        if rs_client.read_param(id, 'mechpos') > 2:
            print("Zeroing error: re-zero hardware.")
            sys.exit(1)

    # Set all motors to velocity mode and enable
    for id in motor_ids:
        rs_client.write_param(id, 'run_mode', robstride.RunMode.Speed)
        rs_client.enable(id)

    # Initialize values
    speed = 0
    direction = np.zeros((3, 1))
    hold_targets = {}

    print("Initialization Completed")

    while True:
        # Gripper logic
        if using_gripper and gamepad_state['open'] == 1:
            gamepad_state['open'] = 0
            gripper.open_step()

        if using_gripper and gamepad_state['close'] == 1:
            gamepad_state['close'] = 0
            gripper.close_step()

        if using_gripper and gamepad_state['toggle'] == 1:
            gamepad_state['toggle'] = 0
            gripper.toggle()

        # Read mechpos
        mech_positions = {id: rs_client.read_param(id, 'mechpos') for id in motor_ids}

        # Compute input direction vector
        x_norm = (gamepad_state['x'] - x_center) / x_halfrange
        y_norm = (gamepad_state['y'] - y_center) / y_halfrange
        x_norm = max(-1.0, min(1.0, x_norm))
        y_norm = max(-1.0, min(1.0, y_norm))

        # deadzone threshold
        thr = 0.1
        direction = np.zeros(3)
        if abs(x_norm) > thr:
            direction[0] = x_norm
        if abs(y_norm) > thr:
            direction[1] = y_norm

        # d‑pad for Z
        hat = gamepad_state['hat_y']
        if hat != 0:
            direction[2] = -hat


        temp_direction = np.zeros((3, 1))
        if kb.key_states['a']:     temp_direction[0] += 1
        if kb.key_states['d']:     temp_direction[0] += -1
        if kb.key_states['w']:     temp_direction[1] += 1
        if kb.key_states['s']:     temp_direction[1] += -1
        if kb.key_states['up']:    temp_direction[2] += 1
        if kb.key_states['down']:  temp_direction[2] += -1

        # Speed control logic
        if np.linalg.norm(temp_direction) < 0.0001:
            speed = max(0, speed - accel / command_rate)
        else:
            direction = temp_direction
            speed = min(max_speed, speed + accel / command_rate)

        # Holding mode
        if speed < 0.0001 and not kb.key_states['1'] and not kb.key_states['2']:
            if not hold_targets:
                hold_targets = {id: mech_positions[id] for id in motor_ids}
            for i, id in enumerate(motor_ids):
                error = hold_targets[id] - mech_positions[id]
                hold_speed = np.clip(Kp_hold * error, -max_hold_speed, max_hold_speed)
                rs_client.write_param(id, 'spd_ref', hold_speed)
        else:
            hold_targets = {}

            # Null space motion
            if speed < 0.0001 and (kb.key_states['1'] or kb.key_states['2']):
                null_vec = scipy.linalg.null_space(jac_pos)[:, 0]
                null_vec /= null_vec[0]
                joint_vels = null_speed * null_vec if kb.key_states['1'] else -null_speed * null_vec

                temp_joint_angles = np.array([mech_positions[id] / motor_ratios[i] for i, id in enumerate(motor_ids)])
                next_qpos = temp_joint_angles + joint_vels / command_rate

                limit_violation = False
                for i in range(len(motor_ids)):
                    lower, upper = joint_limits[i]
                    if next_qpos[i] < lower or next_qpos[i] > upper:
                        limit_violation = True
                        break

                if limit_violation:
                    joint_vels = np.zeros(len(motor_ids))
                    speed = 0
                    print("Blocked: joint limit reached (null space motion).")

            else:
                norm_dir = direction / np.linalg.norm(direction)
                speed_vec = speed * norm_dir

                temp_joint_angles = np.array([mech_positions[id] / motor_ratios[i] for i, id in enumerate(motor_ids)])
                data.qpos = temp_joint_angles
                mj.mj_forward(model, data)

                dist_from_center = site.xpos - reachable_sphere_center
                outside = np.linalg.norm(dist_from_center) > reachable_sphere_radius

                if outside and np.dot(dist_from_center, speed_vec.flatten()) > 0:
                    joint_vels = np.zeros(len(motor_ids))
                    speed = 0
                    print("Blocked: outside reachable zone.")
                else:
                    mj.mj_jacSite(model, data, jac_pos, jac_rot, site.id)
                    joint_vels = (np.linalg.pinv(jac_pos) @ speed_vec).flatten()

                    # Joint limit check
                    next_qpos = temp_joint_angles + joint_vels / command_rate
                    limit_violation = False
                    for i in range(len(motor_ids)):
                        lower, upper = joint_limits[i]
                        if next_qpos[i] < lower or next_qpos[i] > upper:
                            limit_violation = True
                            break

                    if limit_violation:
                        joint_vels = np.zeros(len(motor_ids))
                        speed = 0
                        print("Blocked: joint limit reached.")

                    elif outside:
                        print("Returning inward from boundary")

            for i, id in enumerate(motor_ids):
                rs_client.write_param(id, 'spd_ref', joint_vels[i] * motor_ratios[i])

        time.sleep(1 / command_rate)

