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

motor_ratios = np.array([-1, -1, 1, -3])
motor_ids = [1, 2, 3, 4]
reachable_sphere_center = np.array([0, 0, 0.115])
reachable_sphere_radius = 0.8

# Hybrid hold tuning
Kp_hold = 10.0
max_hold_speed = 2

# Load MuJoCo model
model = mj.MjModel.from_xml_path(urdf_path)
data = mj.MjData(model)
site = data.site(site_name)
jac_pos = np.zeros((3, model.nv))
jac_rot = np.zeros((3, model.nv))

# Key states
key_states = {
    'a': False, 'd': False, 'w': False, 's': False,
    '1': False, '2': False, 'up': False, 'down': False,
}

def on_press(key):
    try:
        if key.char in key_states:
            key_states[key.char] = True
    except AttributeError:
        if key == keyboard.Key.up:
            key_states['up'] = True
        elif key == keyboard.Key.down:
            key_states['down'] = True

def on_release(key):
    try:
        if key.char in key_states:
            key_states[key.char] = False
    except AttributeError:
        if key == keyboard.Key.up:
            key_states['up'] = False
        elif key == keyboard.Key.down:
            key_states['down'] = False

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

with can.Bus() as bus:
    rs_client = robstride.Client(bus)

    # Set all motors to velocity mode and enable
    for id in motor_ids:
        rs_client.write_param(id, 'run_mode', robstride.RunMode.Speed)
        rs_client.enable(id)

    speed = 0
    direction = np.zeros((3, 1))
    hold_targets = {}

    print("Initialization Completed")

    while True:
        # Read mechpos — this makes J2 happy for some reason
        mech_positions = {}
        for id in motor_ids:
            mech_positions[id] = rs_client.read_param(id, 'mechpos')

        # Compute input direction vector
        temp_direction = np.zeros((3, 1))
        if key_states['a']:     temp_direction[0] += 1
        if key_states['d']:     temp_direction[0] += -1
        if key_states['w']:     temp_direction[1] += 1
        if key_states['s']:     temp_direction[1] += -1
        if key_states['up']:    temp_direction[2] += 1
        if key_states['down']:  temp_direction[2] += -1

        # Speed control logic
        if np.linalg.norm(temp_direction) < 0.0001:
            speed = max(0, speed - accel / command_rate)
        else:
            direction = temp_direction
            speed = min(max_speed, speed + accel / command_rate)

        if speed < 0.0001 and not key_states['1'] and not key_states['2']:
            # Entering hold mode — capture current positions
            if not hold_targets:
                for id in motor_ids:
                    hold_targets[id] = mech_positions[id]

            # Hybrid hold using velocity control
            for i, id in enumerate(motor_ids):
                error = hold_targets[id] - mech_positions[id]
                hold_speed = Kp_hold * error
                hold_speed = np.clip(hold_speed, -max_hold_speed, max_hold_speed)
                print(hold_speed)
                rs_client.write_param(id, 'spd_ref', hold_speed)

        else:
            # Clear hold targets (will be re-captured when stopping)
            hold_targets = {}

            # Compute joint velocities from desired end-effector motion
            if speed < 0.0001 and (key_states['1'] or key_states['2']):
                null_vec = scipy.linalg.null_space(jac_pos)[:, 0]
                null_vec /= null_vec[0]
                joint_vels = null_speed * null_vec if key_states['1'] else -null_speed * null_vec
            else:
                norm_dir = direction / np.linalg.norm(direction)
                speed_vec = speed * norm_dir

                temp_joint_angles = np.zeros(len(motor_ids))
                for i, id in enumerate(motor_ids):
                    temp_joint_angles[i] = mech_positions[id] / motor_ratios[i]

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

            # Send velocity commands
            for i, id in enumerate(motor_ids):
                rs_client.write_param(id, 'spd_ref', joint_vels[i] * motor_ratios[i])

        time.sleep(1 / command_rate)
