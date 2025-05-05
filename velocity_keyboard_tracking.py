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

# List of motor ID's for looping through and setting stuff
motor_ids = [1, 2, 3, 4]

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
    for id in motor_ids:
        rs_client.write_param(id, 'run_mode', robstride.RunMode.Position)

    # Enable motors
    for id in motor_ids:
        rs_client.enable(id)  

    # Initialize speed tracker
    speed = 0

    # Keep track of mode to avoid crowding CAN bus with change requests
    mode = 'pos'

    # Keep track of old direction for slowdown events
    direction = np.zeros((3,1))

    # Record initial joint angles
    joint_angles = np.zeros(len(motor_ids))
    for i, id in enumerate(motor_ids):
        joint_angles[i] = rs_client.read_param(id, 'mechpos') / motor_ratios[i]  # FIXED indexing

    print("Initialization Completed")


    while True:
        # Read motor positions, this make J2 happy for some reason
        for id in motor_ids:
            rs_client.read_param(id, 'mechpos')

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

        # If no keys pressed, start slowing down
        if np.linalg.norm(temp_direction) < 0.0001:
            speed = max(0, speed - accel / command_rate)
        else:
            direction = temp_direction
            speed = min(max_speed, speed + accel / command_rate)
        
        # If speed command is 0, change motors to position mode for better placeholding
        # TODO: Make this actually work, for now just sending zero velocities and it works pretty well
        if speed < 0.0001 and not key_states['1'] and not key_states['2']:
            if mode != 'pos':
                mode = 'pos'
                # Update joint angles
                # for i, id in enumerate(motor_ids):
                #     pos = rs_client.read_param(id, 'mechpos')
                #     rs_client.write_param(id, 'loc_ref', pos)
                #     rs_client.write_param(id, 'run_mode', robstride.RunMode.Position)
            for i, id in enumerate(motor_ids):
                rs_client.write_param(id, 'spd_ref', 0 * motor_ratios[i])

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

            # Position control - end effector moves through space
            else:
                # Compute speed vector
                normalized_direction = direction / np.linalg.norm(direction)
                speed_vec = speed * normalized_direction

                # Get motor angles
                temp_joint_angles = np.zeros(len(motor_ids))
                for i, id in enumerate(motor_ids):
                    temp_joint_angles[i] = rs_client.read_param(id, 'mechpos') / motor_ratios[i]  # FIXED indexing

                # Update MJCF and get position jacobian
                data.qpos = temp_joint_angles
                mj.mj_forward(model, data)

                # Check if we've left reachable sphere
                dist_from_center = site.xpos - reachable_sphere_center
                outside = np.linalg.norm(dist_from_center) > reachable_sphere_radius

                if outside:
                    # You're outside, but check direction of motion
                    if np.dot(dist_from_center, speed_vec.flatten()) > 0:
                        # Still trying to go further out — block it
                        joint_vels = np.zeros(len(joint_angles))
                        speed = 0
                        print("Command exceeds reachable zone! Motion blocked")
                    else:
                        # Outside but trying to return — allow it
                        joint_angles = temp_joint_angles
                        mj.mj_jacSite(model, data, jac_pos, jac_rot, site.id)
                        joint_vels = (np.linalg.pinv(jac_pos) @ speed_vec).flatten()
                        print("Outside boundary, returning inward")
                        
                else:
                    # Safe — compute normally
                    joint_angles = temp_joint_angles
                    mj.mj_jacSite(model, data, jac_pos, jac_rot, site.id)
                    joint_vels = (np.linalg.pinv(jac_pos) @ speed_vec).flatten()

                # Check if we've left reachable sphere, reset MJCF, and emergency stop
                dist_from_center = site.xpos - reachable_sphere_center
                if np.linalg.norm(dist_from_center) > reachable_sphere_radius:
                    if np.dot(dist_from_center, speed_vec.flatten()) > 0:
                        # Moving further out — cancel motion
                        joint_vels = np.zeros(len(joint_angles))
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
                for id in motor_ids:
                    rs_client.write_param(id, 'run_mode', robstride.RunMode.Speed)

            # Send motor commands (use ratio to make sure motors behave correctly)
            print(joint_vels)
            for i, id in enumerate(motor_ids):
                rs_client.write_param(id, 'spd_ref', joint_vels[i] * motor_ratios[i])  # FIXED indexingwwwwwww

        time.sleep(1 / command_rate)