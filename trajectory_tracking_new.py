import mujoco as mj
import can
import robstride
import time
import numpy as np
import sys

from robot_control_utils.TrajectoryPlanner import TrajectoryPlanner6dof as trajectory_planner

########################
# CONFIGURATION SECTION
########################

urdf_path = 'robot_models/Sim_Arm_4DOF_May_25/robot.xml'
site_name = 'endeff'
command_rate = 50

# Motor IDs and Ratios
motor_ids = [1, 2, 3, 4]
motor_ratios = [1, 1, -1, -3]

# Reachability sphere
reachable_sphere_center = np.array([0, 0, 0.115])
reachable_sphere_radius = 0.9

########################
# MODEL & TRAJECTORY SETUP
########################

model = mj.MjModel.from_xml_path(urdf_path)
data = mj.MjData(model)
site = data.site(site_name)

traj = trajectory_planner(model, data, site, [0.0, 0.0, 0.0, 0.0], command_rate)
traj.addHold(3)
traj.addLinearMove_3dof(np.array([0, 0.4, 0.1]), 2)
traj.addLinearMove_3dof(np.array([0, 0.7, 0.1]), 2)
traj.addLinearMove_3dof(np.array([0.35, 0.5, 0.2]), 2)
traj.addLinearMove_3dof(np.array([-0.35, 0.5, 0.2]), 2)
traj.addLinearMove_3dof(np.array([0, 0.5, 0.4]), 2)
traj.addLinearMove_3dof(np.array([0, 0.2, 0.8]), 2)

# Apply motor ratios
for i, ratio in enumerate(motor_ratios):
    traj.pos_ref[:, i] *= ratio
    traj.vel_ref[:, i] *= ratio

########################
# TRAJECTORY VALIDATION
########################

# Reachability check
for i, point in enumerate(traj.taskSpaceTraj):
    pos = point[:3]
    dist = np.linalg.norm(pos - reachable_sphere_center)
    if dist > reachable_sphere_radius:
        print(f"[ERROR] Trajectory point {i} at {pos} is outside the reachable sphere (distance = {dist:.3f} m)")
        sys.exit(1)

# Joint limit check
joint_limits = model.jnt_range[:model.njnt]  # (lower, upper)
for i, joint_pos in enumerate(traj.jointSpaceTraj):
    for j in range(len(motor_ids)):
        joint_val = joint_pos[j]
        lower, upper = joint_limits[j]
        if joint_val < lower or joint_val > upper:
            print(f"[ERROR] Trajectory point {i} violates joint limit for joint {j}: {joint_val:.3f} not in [{lower:.3f}, {upper:.3f}]")
            sys.exit(1)

print("[INFO] Trajectory is within reachable bounds and joint limits. Executing...")

########################
# EXECUTION
########################

with can.Bus(interface='socketcan', channel='can0', bitrate=1000000) as bus:
    rs_client = robstride.Client(bus)

    # Check for issue where zeroing leads to values in the 6 range
    for id in motor_ids:
        pos = rs_client.read_param(id, 'mechpos')
        if pos > 1:
            print("Something went wrong with zeroing, please re-zero")
            sys.exit(0)

    # Set motors to operation mode
    for id in motor_ids:
        rs_client.write_param(id, 'run_mode', robstride.RunMode.Operation)
        rs_client.enable(id)

    # Record Start Time
    start_time = time.time()
    last_command_time = start_time

    # Control Loop
    while True:
        current_time = time.time()
        elapsed = current_time - start_time
        step = min(int(np.floor(elapsed * command_rate)), np.shape(traj.jointSpaceTraj)[0] - 1)

        # Check if it's time to send a new command
        if current_time - last_command_time >= 1.0/command_rate:
            for i, id in enumerate(motor_ids):
                # Use control method instead of write_param
                feedback = rs_client.control(
                    motor_id=id,
                    torque=0.0,  # You could add feedforward torque if needed
                    mech_position=traj.pos_ref[step, i],
                    speed=traj.vel_ref[step, i],
                    kp=60.0,
                    kd=3
                )
                
                # Check for errors
                if feedback.errors:
                    print(f"Motor {id} reported errors: {feedback.errors}")
                    # Handle errors appropriately
                
                # Optional: Check tracking performance
                pos_error = abs(feedback.angle - traj.pos_ref[step, i])
                if pos_error > 0.1:  # 0.1 rad threshold
                    print(f"Motor {id} position error: {pos_error:.3f} rad")
            
            last_command_time = current_time

        # Check if trajectory is complete
        if step >= np.shape(traj.jointSpaceTraj)[0] - 1:
            break