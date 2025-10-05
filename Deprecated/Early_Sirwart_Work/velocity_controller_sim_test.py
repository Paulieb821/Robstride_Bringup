import mujoco as mj
import mujoco.viewer
import numpy as np
import time
import scipy
import pynput.keyboard as keyboard

# Config
urdf_path = 'robot_models/Sim_Arm_Back_2foot_noEE/robot.xml'
site_name = 'endeff'
command_rate = 20
max_speed = 6
accel = max_speed*4
null_speed = 6
motor_ratios = np.array([-1, -1, 1, -3])

# Main Arm
# reachable_sphere_center = np.array([0, 0, 0.115])
# reachable_sphere_radius = 0.8
# Back Arm 1 foot
# reachable_sphere_center = np.array([0, 0.12, 0.0])
# reachable_sphere_radius = 0.75
# Back Arm 2 foot
reachable_sphere_center = np.array([0, 0.12, 0.0])
reachable_sphere_radius = 1.05

# Load MuJoCo model
model = mj.MjModel.from_xml_path(urdf_path)
data = mj.MjData(model)
site_id = model.site(site_name).id
jac_pos = np.zeros((3, model.nv))
jac_rot = np.zeros((3, model.nv))

# Key state tracking
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

direction = np.zeros((3,1))
speed = 0

# Viewer setup and simulation loop
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Viewer launched. Use keyboard to control the robot.")

    while viewer.is_running():
        start_time = time.time()
        
        # Control logic
        temp_direction = np.zeros((3,1))
        if key_states['a']: temp_direction[0] += 1
        if key_states['d']: temp_direction[0] -= 1
        if key_states['up']: temp_direction[2] += 1
        if key_states['down']: temp_direction[2] -= 1
        if key_states['w']: temp_direction[1] += 1
        if key_states['s']: temp_direction[1] -= 1

        if np.linalg.norm(temp_direction) < 1e-4:
            speed = max(0, speed - accel / command_rate)
        else:
            direction = temp_direction
            speed = min(max_speed, speed + accel / command_rate)

        if speed < 1e-4 and not (key_states['1'] or key_states['2']):
            data.qvel[:] = 0
        else:
            mj.mj_forward(model, data)
            site_xpos = data.site_xpos[site_id]

            if speed < 1e-4 and (key_states['1'] or key_states['2']):
                mj.mj_jacSite(model, data, jac_pos, jac_rot, site_id)
                null_vec = scipy.linalg.null_space(jac_pos)[:, 0]
                null_vec_unit = null_vec / null_vec[0]
                joint_vels = null_speed * null_vec_unit if key_states['1'] else -null_speed * null_vec_unit
            else:
                normalized_direction = direction / np.linalg.norm(direction)
                speed_vec = speed * normalized_direction

                mj.mj_jacSite(model, data, jac_pos, jac_rot, site_id)
                dist_from_center = site_xpos - reachable_sphere_center
                if np.linalg.norm(dist_from_center) > reachable_sphere_radius:
                    if np.dot(dist_from_center, speed_vec.flatten()) > 0:
                        print("Out of reach. Motion blocked.")
                        joint_vels = np.zeros(model.nv)
                        speed = 0
                    else:
                        joint_vels = (np.linalg.pinv(jac_pos) @ speed_vec).flatten()
                else:
                    joint_vels = (np.linalg.pinv(jac_pos) @ speed_vec).flatten()

            data.qvel[:len(joint_vels)] = joint_vels

        # Step and render
        mj.mj_step(model, data)
        viewer.sync()
        
        # Timing
        time.sleep(max(0, 1/command_rate - (time.time() - start_time)))

# nvidia-smi