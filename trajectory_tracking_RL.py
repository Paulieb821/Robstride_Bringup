import mujoco as mj
import can
import robstride
import time
import numpy as np
import sys
from robot_control_utils.TrajectoryPlanner import TrajectoryPlanner6dof as trajectory_planner
from robot_control_utils.trajectory_logger import TrajectoryLogger

########################
# CONFIGURATION SECTION
########################

urdf_path = 'robot_models/Sim_Arm_4DOF_May_25/robot.xml'
site_name = 'endeff'
command_rate = 100

# Motor IDs and Ratios
kp_arr = np.array([300.0, 700.0, 300.0, 300.0])
kd_arr = np.array([50.0, 10.0, 50.0, 10.0])
# kp_arr = np.array([20.0, 40.0, 20.0, 20.0])
# kd_arr = np.array([10.0, 20.0, 10.0, 10.0])

motor_ids = [1,2,3,4]
motor_ratios = [1,1,-1,-3]

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

# # Classic Trajectory
# traj.addHold(5)
# duration = 3
# traj.addLinearMove_3dof(np.array([0, 0.4, 0.1]), duration)
# traj.addLinearMove_3dof(np.array([0, 0.7, 0.1]), duration)
# traj.addLinearMove_3dof(np.array([0.35, 0.5, 0.2]), duration)
# traj.addLinearMove_3dof(np.array([-0.35, 0.5, 0.2]), duration)
# traj.addLinearMove_3dof(np.array([0, 0.5, 0.4]), duration)
# traj.addLinearMove_3dof(np.array([0, 0.5, 0.7]), duration)
# traj.addHold(5)

# Vertical Square Pattern
# traj.addHold(5)
# duration = 3
# traj.addLinearMove_3dof(np.array([-0.3, 0.4, 0.5]), 2)
# for i in range(10):
#     traj.addLinearMove_3dof(np.array([-0.3, 0.4, -0.1]), duration)
#     traj.addLinearMove_3dof(np.array([0.3, 0.4, -0.1]), duration)
#     traj.addLinearMove_3dof(np.array([0.3, 0.4, 0.5]), duration)
#     traj.addLinearMove_3dof(np.array([-0.3, 0.4, 0.5]), duration)
# traj.addHold(5)


# Horizontal Square Pattern
traj.addHold(5)
duration = 3 #(3, 1.5, 1)
traj.addLinearMove_3dof_fixed_joint(np.array([-0.15, 0.4, 0.3]), 2)
for i in range(10):
    traj.addLinearMove_3dof_fixed_joint(np.array([-0.15, 0.7, 0.3]), duration)
    traj.addLinearMove_3dof_fixed_joint(np.array([0.15, 0.7, 0.3]), duration)
    traj.addLinearMove_3dof_fixed_joint(np.array([0.15, 0.4, 0.3]), duration)
    traj.addLinearMove_3dof_fixed_joint(np.array([-0.15, 0.4, 0.3]), duration)
traj.addHold(5)


# Circle Pattern
# radius = 0.3
# circle_start = np.array([0.0, 0.5, 0.0])
# duration = 18.8 #(18.8, 9.4, 6.27)

# traj.addHold(5)
# traj.addLinearMove_3dof(circle_start, 2)
# for i in range(10):
#     traj.trace_circle(radius, duration)
# traj.addHold(5)

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
            print(f"[ERROR] Trajectory point {i} violates joint limit for joint {j+1}: {joint_val:.3f} not in [{lower:.3f}, {upper:.3f}]")
            sys.exit(1)

print("[INFO] Trajectory is within reachable bounds and joint limits. Executing...")

########################
# EXECUTION
########################

with can.Bus(interface='socketcan', channel='can0', bitrate=1000000) as bus:
    rs_client = robstride.Client(bus)
    # Initialize logger
    logger = TrajectoryLogger(motor_ids, kp=kp_arr[0],kp2=kp_arr[1], kd=kd_arr[0], kd2=kd_arr[1], save_dir="tracking_tests")

    # Check for issue where zeroing leads to values in the 6 range
    for id in motor_ids:
        motor_model = 1 if id != 2 else 2
        pos = rs_client.read_param(id, 'mechpos')
        if pos > 1:
            print("Something went wrong with zeroing, please re-zero")
            sys.exit(0)

    # Set motors to operation mode
    for id in motor_ids:
        motor_model = 1 if id != 2 else 2
        rs_client.write_param(id, 'run_mode', robstride.RunMode.Operation, motor_model=motor_model)
        rs_client.enable(id, motor_model=motor_model)

    # Record Start Time
    start_time = time.time()
    last_command_time = start_time

    # Control Loop
    while True:
        current_time = time.time()
        elapsed = current_time - start_time
        step = min(int(np.floor(elapsed * command_rate)), np.shape(traj.jointSpaceTraj)[0] - 1)

        # Check if it's time to send a new command
        feedback_data = {}
        ref_data = {}
        
        for i, id in enumerate(motor_ids):
            # Use control method instead of write_param
            motor_model = 1 if id != 2 else 2
            feedback = rs_client.control(
                motor_id=id,
                torque=0.0,  # You could add feedforward torque if needed
                mech_position=traj.pos_ref[step, i],
                speed=traj.vel_ref[step,i],  # Use zero speed for position control
                kp=kp_arr[i],
                kd=kd_arr[i],
                motor_model=motor_model
            )
            
            # Store feedback data
            feedback_data[id] = {
                'angle': feedback.angle,
                'velocity': feedback.velocity,
                'torque': feedback.torque,
                'errors': feedback.errors
            }
            
            # Store reference data
            ref_data[id] = {
                'position': traj.pos_ref[step, i],
                'velocity': traj.vel_ref[step, i]
            }
            
            # Check for errors
            if feedback.errors:
                print(f"Motor {id} reported errors: {feedback.errors}")
        
        # Log data
        logger.log_data(elapsed, feedback_data, ref_data)
        last_command_time = current_time

        # Check if trajectory is complete
        if step >= np.shape(traj.jointSpaceTraj)[0] - 1:
            break

    # Finalize logging
    logger.finalize()
    print(f"[INFO] Trajectory execution complete. Logs saved in {logger.run_dir}")