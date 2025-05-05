import mujoco as mj
import mujoco.viewer
import time
import numpy as np
import scipy.linalg
import pynput.keyboard as keyboard

# --- MuJoCo configuration ---
model_path = 'robot_models/Sim_Arm_4DOF_Mar_25/robot.xml'
site_name = 'endeff'
command_rate = 20  # Hz
max_speed = 0.2
accel = 0.4
null_speed = 0.2
Kp_hold = 10.0
max_hold_speed = 2.0
reachable_sphere_center = np.array([0, 0, 0.115])
reachable_sphere_radius = 0.8

# --- Load MuJoCo model ---
model = mj.MjModel.from_xml_path(model_path)
data = mj.MjData(model)
site_id = model.site(site_name).id
jac_pos = np.zeros((3, model.nv))
jac_rot = np.zeros((3, model.nv))

# --- Viewer setup ---
with mujoco.viewer.launch_passive(model, data) as viewer:

    # --- Keyboard setup ---
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

    print("MuJoCo viewer started. Use WASD + arrow keys to move end-effector. 1/2 to explore null space.")

    # --- Control loop ---
    speed = 0
    direction = np.zeros((3, 1))
    hold_targets = None

    last_time = time.time()
    while viewer.is_running():
        current_time = time.time()
        dt = current_time - last_time
        if dt < 1 / command_rate:
            time.sleep(1 / command_rate - dt)
        last_time = current_time

        # Compute direction from keys
        temp_direction = np.zeros((3, 1))
        if key_states['a']:     temp_direction[0] += 1
        if key_states['d']:     temp_direction[0] -= 1
        if key_states['w']:     temp_direction[1] += 1
        if key_states['s']:     temp_direction[1] -= 1
        if key_states['up']:    temp_direction[2] += 1
        if key_states['down']:  temp_direction[2] -= 1

        # Speed control
        if np.linalg.norm(temp_direction) < 1e-4:
            speed = max(0, speed - accel / command_rate)
        else:
            direction = temp_direction
            speed = min(max_speed, speed + accel / command_rate)

        # Hold logic
        if speed < 1e-4 and not key_states['1'] and not key_states['2']:
            if hold_targets is None:
                hold_targets = np.copy(data.qpos)
            errors = hold_targets - data.qpos
            vel_cmd = Kp_hold * errors
            vel_cmd = np.clip(vel_cmd, -max_hold_speed, max_hold_speed)
            data.ctrl[:] = vel_cmd
        else:
            hold_targets = None
            if speed < 1e-4 and (key_states['1'] or key_states['2']):
                mj.mj_jacSite(model, data, jac_pos, jac_rot, site_id)
                null_vecs = scipy.linalg.null_space(jac_pos)
                if null_vecs.size == 0:
                    joint_vels = np.zeros(model.nv)
                else:
                    null_vec = null_vecs[:, 0]
                    null_vec /= null_vec[0] if null_vec[0] != 0 else 1
                    joint_vels = null_speed * null_vec if key_states['1'] else -null_speed * null_vec
            else:
                norm_dir = direction / np.linalg.norm(direction)
                speed_vec = speed * norm_dir

                mj.mj_forward(model, data)
                end_effector_pos = data.site_xpos[site_id]
                dist_from_center = end_effector_pos - reachable_sphere_center
                outside = np.linalg.norm(dist_from_center) > reachable_sphere_radius

                if outside and np.dot(dist_from_center, speed_vec.flatten()) > 0:
                    joint_vels = np.zeros(model.nv)
                    speed = 0
                    print("Blocked: outside reachable zone.")
                else:
                    mj.mj_jacSite(model, data, jac_pos, jac_rot, site_id)
                    joint_vels = (np.linalg.pinv(jac_pos) @ speed_vec).flatten()
                    if outside:
                        print("Returning inward from boundary")

            data.ctrl[:] = joint_vels

        mj.mj_step(model, data)
