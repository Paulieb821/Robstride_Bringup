import mujoco as mj
import can
import robstride
import time
import numpy as np
import pynput.keyboard as keyboard

from robot_control_utils.TrajectoryPlanner import TrajectoryPlanner6dof as trajectory_planner
from robot_control_utils.NRIK import NRIK

# Mujoco configuration
urdf_path = 'robot_models/Sim_Arm_4DOF_Mar_25/robot.xml'
site_name = 'endeff'
command_rate = 20
speed = 0.2
max_velocity = speed / command_rate
acceleration = 0.1 * max_velocity  # Tweak this for faster or slower ramp-up/down

# Load the MuJoCo model and data
model = mj.MjModel.from_xml_path(urdf_path)
data = mj.MjData(model)
site = data.site(site_name)
ik = NRIK(model, data, site)

# Get initial position of arm
mj.mj_forward(model, data)
endeff_pos = site.xpos.copy()
joint_pos = ik.solveIK_3dof(endeff_pos)

# Velocity smoothing
velocity = np.zeros(3)

# Key states
key_states = {
    'a': False,
    'd': False,
    'w': False,
    's': False,
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

    rs_client.write_param(1, 'run_mode', robstride.RunMode.Position)
    rs_client.write_param(2, 'run_mode', robstride.RunMode.Position)
    rs_client.write_param(3, 'run_mode', robstride.RunMode.Position)
    rs_client.write_param(4, 'run_mode', robstride.RunMode.Position)

    rs_client.enable(1)
    rs_client.enable(2)
    rs_client.enable(3)
    rs_client.enable(4)

    while True:
        # Determine direction
        direction = np.zeros(3)
        if key_states['a']:
            direction += np.array([1, 0, 0])
        if key_states['d']:
            direction += np.array([-1, 0, 0])
        if key_states['up']:
            direction += np.array([0, 0, 1])
        if key_states['down']:
            direction += np.array([0, 0, -1])
        if key_states['w']:
            direction += np.array([0, 1, 0])
        if key_states['s']:
            direction += np.array([0, -1, 0])

        if np.linalg.norm(direction) > 0:
            direction = direction / np.linalg.norm(direction)
            target_velocity = direction * max_velocity
        else:
            target_velocity = np.zeros(3)

        # Smooth acceleration/deceleration
        delta_v = target_velocity - velocity
        delta_v = np.clip(delta_v, -acceleration, acceleration)
        velocity += delta_v

        # Update position
        endeff_pos += velocity

        # Solve IK
        joint_pos = ik.solveIK_3dof(endeff_pos)
        print("End Effector:", np.round(endeff_pos, 2), "Joints:", np.round(joint_pos, 2))

        # Send motor commands
        rs_client.write_param(1, 'loc_ref', -joint_pos[0])
        rs_client.write_param(2, 'loc_ref', -joint_pos[1])
        rs_client.write_param(3, 'loc_ref', joint_pos[2])
        rs_client.write_param(4, 'loc_ref', -3 * joint_pos[3])

        time.sleep(1 / command_rate)

        #swwwwwwwwwwwwwww