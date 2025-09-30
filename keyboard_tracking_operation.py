import mujoco as mj
import can
import robstride
import time
import numpy as np
import serial
import sys

from UI_utils.keyboard_listener import Keyboard_Listener
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
acceleration = 0.5 * max_velocity  # Smoothing

# Motor mapping
motor_ids = [1, 2, 3, 4]
motor_ratios = [1, 1, -1, -3]

# PD Gains
kp_arr = np.array([50.0, 100.0, 50.0, 50.0]) # old 
kd_arr = np.array([10.0, 20.0, 10.0, 1.0]) # old 
# kp_arr = np.array([20.0, 40.0, 20.0, 20.0])
# kd_arr = np.array([10.0, 20.0, 10.0, 10.0])

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

# Joint limits
joint_limits = model.jnt_range[:model.njnt]  # (lower, upper)

# Control helpers
velocity = np.zeros(3)
kb = Keyboard_Listener()

# Gripper setup
if using_gripper:
    try:
        gripper = GripperController(            
            port="/dev/ttyUSB0",
            baud=9600,
            servo_ids=(8, 11),     # DOUBLE CHECK WHAT IS ON HARDWARE
            pulse_open = 500,    # swap these if direction is reversed
            pulse_closed = 2000
        )

    except Exception as e:
        print(f"[WARNING] Gripper init failed: {e}")
        using_gripper = False

########################
# MAIN CONTROL LOOP
########################

with can.Bus(interface='socketcan', channel='can0', bitrate=1000000) as bus:
    rs_client = robstride.Client(bus)

    # Check for issue where zeroing leads to values in the 6 range
    for id in motor_ids:
        pos = rs_client.read_param(id, 'mechpos')
        if pos > 5:
            print("Something went wrong with zeroing, please re-zero")
            sys.exit(0)

    # Set and enable all motors
    for id in motor_ids:
        rs_client.write_param(id, 'run_mode', robstride.RunMode.Operation)
        rs_client.enable(id)
    
    while True:
       
        if using_gripper:        
            if kb.key_states['o']:
                kb.key_states['o'] = False
                gripper.open_step()
            elif kb.key_states['c']:
                kb.key_states['c'] = False
                gripper.close_step()
            elif kb.key_states['space']:
                kb.key_states['space'] = False
                gripper.toggle()
            
        
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
        
        # Inverse kinematics
        proposed_joint_pos = ik.solveIK_3dof(proposed_pos)

        # Joint limit check
        joint_limit_violation = False
        for i in range(len(motor_ids)):
            lower, upper = joint_limits[i]
            if proposed_joint_pos[i] < lower or proposed_joint_pos[i] > upper:
                joint_limit_violation = True
                break

        if joint_limit_violation:
            print("Blocked: joint limit reached.")
            continue

        # If no violations, accept motion
        endeff_pos = proposed_pos
        joint_pos = proposed_joint_pos

        # print("End Effector:", np.round(endeff_pos, 2), "Joints:", np.round(joint_pos, 2))

        # Send joint positions
        for i, id in enumerate(motor_ids):
            motor_model = 1 if id != 2 else 2
            feedback = rs_client.control(
                motor_id =id,
                torque =0.0,  # You could add feedforward torque if needed
                mech_position = joint_pos[i] * motor_ratios[i],
                speed= 0.0,  # Use zero speed for position control
                kp=kp_arr[i],
                kd=kd_arr[i],
                motor_model=motor_model
            )

        time.sleep(1 / command_rate)

