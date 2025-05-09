import mujoco as mj
import can
import robstride
import time
import numpy as np
import serial
import sys

from UI_utils.keyboard_listener import Keyboard_Listener
from robot_control_utils.NRIK import NRIK

########################
# CONFIGURATION SECTION
########################

urdf_path = 'robot_models/Sim_Arm_4DOF_May_25/robot.xml'
site_name = 'endeff'
command_rate = 20
speed = 0.2
max_velocity = speed / command_rate
acceleration = 0.1 * max_velocity  # Smoothing

# Motor mapping
motor_ids = [1, 2, 3, 4]
motor_ratios = [1, 1, -1, -3]

# Reachability sphere
reachable_sphere_center = np.array([0, 0, 0.115])
reachable_sphere_radius = 0.9

# Gripper control
using_gripper = True

########################
# INITIALIZATION
########################

# Load MuJoCo and IK
model = mj.MjModel.from_xml_path(urdf_path)
data = mj.MjData(model)
site = data.site(site_name)
ik = NRIK(model, data, site)
mj.mj_forward(model, data)
endeff_pos = site.xpos.copy()
joint_pos = ik.solveIK_3dof(endeff_pos)

# Control helpers
velocity = np.zeros(3)
kb = Keyboard_Listener()

# Gripper setup
if using_gripper:
    try:
        PORT = '/dev/ttyACM0'
        BAUD = 115200
        ser = serial.Serial(PORT, BAUD, timeout=0)
        gripper_open = False
        time.sleep(2)
    except serial.SerialException as e:
        print(f"[WARNING] Could not connect to gripper: {e}")
        using_gripper = False

########################
# MAIN CONTROL LOOP
########################

with can.Bus() as bus:
    rs_client = robstride.Client(bus)

    # Check for issue where zeroing leads to values in the 6 range
    for id in motor_ids:
        pos = rs_client.read_param(id, 'mechpos')
        if pos > 5:
            print("Something went wrong with zeroing, please re-zero")
            sys.exit(0)

    # Set and enable all motors
    for id in motor_ids:
        rs_client.write_param(id, 'run_mode', robstride.RunMode.Position)
        rs_client.enable(id)

    while True:
        # Gripper control
        if using_gripper:
            try:
                if kb.key_states['space']:
                    kb.key_states['space'] = False
                    gripper_open = not gripper_open
                    ser.write(b'G' if gripper_open else b'H')

                elif kb.key_states['o']:
                    kb.key_states['o'] = False
                    ser.write(b'o')

                elif kb.key_states['c']:
                    kb.key_states['c'] = False
                    ser.write(b'c')
            except serial.SerialException as e:
                print(f"[ERROR] Gripper communication failed: {e}")
                using_gripper = False

        # Direction input
        direction = np.zeros(3)
        if kb.key_states['a']:     direction += np.array([1, 0, 0])
        if kb.key_states['d']:     direction += np.array([-1, 0, 0])
        if kb.key_states['w']:     direction += np.array([0, 1, 0])
        if kb.key_states['s']:     direction += np.array([0, -1, 0])
        if kb.key_states['up']:    direction += np.array([0, 0, 1])
        if kb.key_states['down']:  direction += np.array([0, 0, -1])

        # Velocity smoothing
        if np.linalg.norm(direction) > 0:
            direction = direction / np.linalg.norm(direction)
            target_velocity = direction * max_velocity
        else:
            target_velocity = np.zeros(3)

        delta_v = target_velocity - velocity
        delta_v = np.clip(delta_v, -acceleration, acceleration)
        velocity += delta_v

        # Position update
        proposed_pos = endeff_pos + velocity

        # Reachability sphere check
        dist_from_center = proposed_pos - reachable_sphere_center
        if np.linalg.norm(dist_from_center) > reachable_sphere_radius:
            print("Blocked: outside reachable zone.")
            continue
        else:
            endeff_pos = proposed_pos

        # Inverse kinematics
        joint_pos = ik.solveIK_3dof(endeff_pos)
        print("End Effector:", np.round(endeff_pos, 2), "Joints:", np.round(joint_pos, 2))

        # Send joint positions
        for i, id in enumerate(motor_ids):
            pos = joint_pos[i] * motor_ratios[i]
            rs_client.write_param(id, 'loc_ref', pos)

        time.sleep(1 / command_rate)
