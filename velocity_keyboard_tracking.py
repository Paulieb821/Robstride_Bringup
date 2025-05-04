import mujoco as mj
import can
import robstride
import time
import numpy as np
import scipy
import pynput.keyboard as keyboard
import scipy.linalg

# Mujoco configuration
urdf_path = 'robot_models/Sim_Arm_4DOF_Mar_25/robot.xml'
site_name = 'endeff'

# Control settings
command_rate = 20   # Hz
max_speed = 0.2     # Speed of end effector m/s
accel = 0.4         # Acceleration on start/stop to avoid vibration
null_speed = 0.2    # Speed at which J1 moves through the null space rad/s

# Flip and gear ratio factors between joints and MJCF
motor_ratios = np.array([-1, -1, 1, -3])

# Coordinates and size of maximum reachable sphere (determined offline for now)
reachable_sphere_center = np.array([0, 0, 0.115])
reachable_sphere_radius = 0.8

# Load the MuJoCo model and data, set up Jacobian
model = mj.MjModel.from_xml_path(urdf_path)
data = mj.MjData(model)
site = data.site(site_name)
jac_pos = np.zeros((3, model.nv))
jac_rot = np.zeros((3, model.nv))

# Key states
key_states = {
    'a': False,
    'd': False,
    'w': False,
    's': False,
    '1': False,
    '2': False,
    'up': False,
    'down': False,
}

def on_press(key):
    try:
        if key.char == 'a':
            key_states['a'] = True
        elif key.char == 'd':
            key_states['d'] = True
        elif key.char == 'w':
            key_states['w'] = True
        elif key.char == 's':
            key_states['s'] = True
        elif key.char == '1':
            key_states['1'] = True
        elif key.char == '2':
            key_states['2'] = True
    except AttributeError:
        if key == keyboard.Key.up:
            key_states['up'] = True
        elif key == keyboard.Key.down:
            key_states['down'] = True

def on_release(key):
    try:
        if key.char == 'a':
            key_states['a'] = False
        elif key.char == 'd':
            key_states['d'] = False
        elif key.char == 'w':
            key_states['w'] = False
        elif key.char == 's':
            key_states['s'] = False
        elif key.char == '1':
            key_states['1'] = False
        elif key.char == '2':
            key_states['2'] = False
    except AttributeError:
        if key == keyboard.Key.up:
            key_states['up'] = False
        elif key == keyboard.Key.down:
            key_states['down'] = False

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# Run Trajectory
with can.Bus() as bus:
    rs_client = robstride.Client(bus)

    # Initialize joints in position mode
    rs_client.write_param(1, 'run_mode', robstride.RunMode.Position)
    rs_client.write_param(2, 'run_mode', robstride.RunMode.Position)
    rs_client.write_param(3, 'run_mode', robstride.RunMode.Position)
    rs_client.write_param(4, 'run_mode', robstride.RunMode.Position)

    rs_client.enable(1)
    rs_client.enable(2)
    rs_client.enable(3)
    rs_client.enable(4)

    # Initialize speed tracker
    speed = 0

    # Keep track of mode to avoid crowding CAN bus with change requests
    mode = 'pos'

    # Keep track of old direction for slowdown events
    direction = np.zeros((3,1))

    # Record initial joint angles
    joint_angles = np.zeros(4)
    joint_angles[0] = rs_client.read_param(1, 'mech_pos') / motor_ratios[0]
    joint_angles[1] = rs_client.read_param(2, 'mech_pos') / motor_ratios[1]
    joint_angles[2] = rs_client.read_param(3, 'mech_pos') / motor_ratios[2]
    joint_angles[3] = rs_client.read_param(4, 'mech_pos') / motor_ratios[3]
    

    while True:
        # Compute direction
        temp_direction = np.zeros((3,1))
        if key_states['a']:
            temp_direction[0] += 1
        if key_states['d']:
            temp_direction[0] += -1
        if key_states['up']:
            temp_direction[2] += 1
        if key_states['down']:
            temp_direction[2] += -1
        if key_states['w']:
            temp_direction[1] += 1
        if key_states['s']:
            temp_direction[1] += -1

        # If no keys pressed, start slowing dow
        if np.linalg.norm(temp_direction) < 0.0001:
            speed = max(0, speed - accel / command_rate)
        else:
            direction = temp_direction
            speed = min(max_speed, speed + accel / command_rate)
        
        # If speed command is 0, change motors to position mode for better placeholding
        if speed < 0.0001 and not key_states['1'] and not key_states['2']:
            if mode != 'pos':
                mode = 'pos'
                rs_client.write_param(1, 'run_mode', robstride.RunMode.Position)
                rs_client.write_param(2, 'run_mode', robstride.RunMode.Position)
                rs_client.write_param(3, 'run_mode', robstride.RunMode.Position)
                rs_client.write_param(4, 'run_mode', robstride.RunMode.Position)

        # Otherwise send out velocity command
        else:
            # Null space control - arm reorients but endeff doesn't move
            if speed < 0.0001 and (key_states['1'] or key_states['2']):
                null_vec = scipy.linalg.null_space(jac_pos)[:,0]
                null_vec_unit_j1_speed = null_vec / null_vec[0]
                if key_states['1']:
                    joint_vels = null_speed * null_vec_unit_j1_speed
                else:
                    joint_vels = -null_speed * null_vec_unit_j1_speed

            # Position contol - end effector moves through space
            else:
                # Compute speed vector
                normalized_direction = direction / np.linalg.norm(direction)
                speed_vec = speed * normalized_direction

                # Get motor angles
                temp_joint_angles = np.zeros(4)
                temp_joint_angles[0] = rs_client.read_param(1, 'mech_pos') / motor_ratios[0]
                temp_joint_angles[1] = rs_client.read_param(2, 'mech_pos') / motor_ratios[1]
                temp_joint_angles[2] = rs_client.read_param(3, 'mech_pos') / motor_ratios[2]
                temp_joint_angles[3] = rs_client.read_param(4, 'mech_pos') / motor_ratios[3]

                # Update MJCF and get position jacobian
                data.qpos = temp_joint_angles
                mj.mj_forward(model, data)

                # Check if we've left reachable sphere, reset MJCF, and emergency stop
                dist_from_center = site.xpos - reachable_sphere_center
                if np.linalg.norm(dist_from_center) > reachable_sphere_radius:
                    if np.dot(dist_from_center, speed_vec.flatten()) > 0:
                        # Moving further out — cancel motion
                        joint_vels = np.zeros(joint_angles.size)
                        speed = 0
                        print("Command exceeds reachable zone! Motion blocked")

                # If location is safe, compute jacobian and calculate minimum-norm joint velocities
                else:
                    joint_angles = temp_joint_angles
                    mj.mj_jacSite(model, data, jac_pos, jac_rot, site.id)
                    joint_vels = (np.linalg.pinv(jac_pos) @ speed_vec).flatten()

            # Change motor mode to velocity
            if mode != 'vel':
                mode = 'vel'
                rs_client.write_param(1, 'run_mode', robstride.RunMode.Speed)
                rs_client.write_param(2, 'run_mode', robstride.RunMode.Speed)
                rs_client.write_param(3, 'run_mode', robstride.RunMode.Speed)
                rs_client.write_param(4, 'run_mode', robstride.RunMode.Speed)

            # Send motor commands (use ratio to make sure motors behave correctly)
            rs_client.write_param(1, 'spd_ref', joint_vels[0] * motor_ratios[0])
            rs_client.write_param(2, 'spd_ref', joint_vels[1] * motor_ratios[1])
            rs_client.write_param(3, 'spd_ref', joint_vels[2] * motor_ratios[2])
            rs_client.write_param(4, 'spd_ref', joint_vels[3] * motor_ratios[3])

        time.sleep(1 / command_rate)