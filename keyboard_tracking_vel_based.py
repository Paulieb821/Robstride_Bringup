import mujoco as mj
import can
import robstride
import time
import numpy as np
import scipy
import scipy.linalg
from UI_utils.keyboard_listener import Keyboard_Listener
import serial
import sys

########################
# EDIT THIS PART - BEGIN
########################

# Mujoco configuration
urdf_path = 'robot_models/Sim_Arm_4DOF_May_25/robot.xml'
site_name = 'endeff'

# Control settings
command_rate = 10   # Hz
max_speed = 0.2     # Speed of end effector m/s
accel = 0.6         # Acceleration on start/stop to avoid vibration
null_speed = 0.2    # Speed at which J1 moves through the null space rad/s

# Motor Setup
motor_ratios = np.array([1, 1, -1, -3]) 
motor_ids = [1, 2, 3, 4]

# Reachability
reachable_sphere_center = np.array([0, 0, 0.115])
reachable_sphere_radius = 0.9       

# Hybrid hold tuning
Kp_hold = 10.0
max_hold_speed = 2

# Gripper
using_gripper = True

######################
# EDIT THIS PART - END
######################

# Load MuJoCo model
model = mj.MjModel.from_xml_path(urdf_path)
data = mj.MjData(model)
site = data.site(site_name)
jac_pos = np.zeros((3, model.nv))
jac_rot = np.zeros((3, model.nv))

# Initialize key tracker
kb = Keyboard_Listener()

# Initialize gripper if needed
if using_gripper:
    try:
        PORT = '/dev/ttyACM0'
        BAUD = 115200
        ser = serial.Serial(PORT, BAUD, timeout=0)
        gripper_open = False
        time.sleep(2)  # Give time to initialize
    except serial.SerialException as e:
        print(f"[WARNING] Could not connect to gripper: {e}")
        using_gripper = False

with can.Bus() as bus:
    rs_client = robstride.Client(bus)

    # Check for issue where zeroing leads to values in the 6 range
    for id in motor_ids:
        pos = rs_client.read_param(id, 'mechpos')
        if pos > 5:
            print("Something went wrong with zeroing, please re-zero")
            sys.exit(0)

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

        # Read mechpos
        mech_positions = {id: rs_client.read_param(id, 'mechpos') for id in motor_ids}

        # Compute input direction vector
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
                    if outside:
                        print("Returning inward from boundary")

            for i, id in enumerate(motor_ids):
                rs_client.write_param(id, 'spd_ref', joint_vels[i] * motor_ratios[i])

        time.sleep(1 / command_rate)
