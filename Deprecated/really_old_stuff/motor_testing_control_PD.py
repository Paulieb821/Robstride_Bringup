# motor_testing_control.py script
import argparse
import math
import time
import os
import threading
import matplotlib.pyplot as plt
import numpy as np
from controller_utils.PD_gain_calculator import pd_gains
from actuator import RobstrideActuator, RobstrideActuatorConfig, RobstrideActuatorCommand, RobstrideConfigureRequest

# Cleanup
# os.system('clear')

# Create Supervisor
supervisor = RobstrideActuator(ports=['/dev/ttyUSB0'], py_actuators_config=[
    (1, RobstrideActuatorConfig(1)),    # J1
    (7, RobstrideActuatorConfig(3)),    # J2
    (3, RobstrideActuatorConfig(1)),    # J3
    ])


supervisor.run_main_loop(1)

# Startup sequence
print("Startup sequence")
time.sleep(1)
supervisor.enable(1)
supervisor.enable(7)
supervisor.enable(3)
print("Motor enabled")

initials_gotten = False
while not initials_gotten:
    state = supervisor.get_actuators_state([1])
    if state:
        initial_pos = math.radians(state[0].position)
        initial_vel = math.radians(state[0].velocity)
        initials_gotten = True


# Trajectorty Parameters
setpoint = 2
move_time = 1

# Transfer function 
b1 = -6.116
b0 = 152.51
a1 = 0.66
a0 = 0.0

# Poles and time step
lambda_val = 20
T = 0.01
manual_correction = np.array([1.0, 1.8])   # [Kp, Kd], adjusting Kd to 2-4X usually works well

# DT Control Parameters
K, L, Ad, Bd, Cd = pd_gains(lambda_val, T, b1, b0, a1, a0)
K = K[0] * manual_correction
L = L[0]

# Motors and Logging
motor_ids = [1, 7, 3]
data = {motor_id: {"time": [], "position": [], "velocity": [], "torque": [], "pos_ref":[], "vel_ref":[], "vel_pred":[], "pos_pred":[]} for motor_id in motor_ids}

# Trajectorty Generation
def cubic_trajectory(xo, xf, tf, time):
    if time < tf:
        c0 = xo
        c2 = 3*(xf - xo)/pow(tf,2)
        c3 = -2*(xf - xo)/pow(tf,3)
        #return (c0 + c2*pow(time,2) + c3*pow(time,3), 2*c2*time + 3*c3*pow(time,2), 2*c2 + 6*c3*time)
        return c0 + c2*pow(time,2) + c3*pow(time,3)
    else:
        #return (xf, 0, 0)
        return xf

def quintic_trajectory(xo, xf, tf, time):
    # Calculate the coefficients for the quintic polynomial
    a0 = xo
    a1 = 0  # initial velocity is 0
    a2 = 0  # initial acceleration is 0
    a3 = (10 * (xf - xo)) / (tf ** 3)
    a4 = (-15 * (xf - xo)) / (tf ** 4)
    a5 = (6 * (xf - xo)) / (tf ** 5) 

    if time < tf:
        # Compute position and velocity at a given time
        position = a0 + a1 * time + a2 * time**2 + a3 * time**3 + a4 * time**4 + a5 * time**5
        velocity = a1 + 2 * a2 * time + 3 * a3 * time**2 + 4 * a4 * time**3 + 5 * a5 * time**4
        accel = 2*a2 + 6*a3*time + 12*a4*time**2 + 20*a5*time**3
        #return position, velocity, accel
        return position
    else:
        # At the final time, return the final position and velocity = 0
        #return xf, 0,0 
        return xf

def generateReferences(x0, xf, move_time, time_step, max_time):
    pos_ref = np.array([quintic_trajectory(x0, xf, move_time, t) for t in np.linspace(0, max_time, int(max_time/time_step))])
    numPoints = np.size(pos_ref)
    vel_ref = np.zeros(numPoints)
    acc_ref = np.zeros(numPoints)
    # Useful constants
    pps = 1/time_step
    a = -2*pps
    b = 2*pow(pps,2)
    T = time_step
    # Solve for velocity and force references
    for i in range(numPoints-1):
        acc_ref[i] = a*vel_ref[i] + b*(pos_ref[i+1] - pos_ref[i])
        vel_ref[i+1] = vel_ref[i] + T*acc_ref[i]
    # Add to Trajectory 
    return pos_ref, vel_ref, acc_ref

# Misc
max_time = move_time + 3
failure_count = 0

# Main Control Loop
def control_thread():
    # Get program start time
    start_time = time.time()
    # State Estimator setup
    xhat = np.array([[initial_pos], [initial_vel]])
    # Get references
    pos_ref_traj, vel_ref_traj, acc_ref_traj = generateReferences(initial_pos, setpoint, move_time, T, max_time)
    # Step Counter
    step_cnt = 0
    # Control loop
    while True:
        # Get elapsed time and check for shutdown
        elapsed = time.time() - start_time
        if (elapsed > max_time):
            supervisor.command_actuators(
                [RobstrideActuatorCommand(actuator_id=1, position=0.0, velocity=0.0, torque=0)])
            supervisor.disable(1)
            supervisor.disable(7)
            supervisor.disable(3)
            break
        # Get trajectory references
        if step_cnt < len(pos_ref_traj):
            pos_ref = pos_ref_traj[step_cnt]
            vel_ref = vel_ref_traj[step_cnt]
            acc_ref = acc_ref_traj[step_cnt]
            step_cnt += 1
        else:
            pos_ref = pos_ref_traj[-1]
            vel_ref = vel_ref_traj[-1]
            acc_ref = acc_ref_traj[-1]
        # Get motor state
        state = supervisor.get_actuators_state([1])
        if state:
            # Process position and velocity
            pos = math.radians(state[0].position)
            vel = math.radians(state[0].velocity)
            # Calculate torque using PD controller
            cmd_trq = K[0]*(pos_ref-xhat[0,0]) + K[1]*(vel_ref-xhat[1,0])
            # Update Estimator and Integrator
            xhat = Ad @ xhat + Bd * cmd_trq - L * (Cd @ xhat - pos)
            #Position Limits
            if abs(pos) > 3.1:
                cmd_trq = 0
            # Send out command
            supervisor.command_actuators(
                [RobstrideActuatorCommand(actuator_id=1, position=0, velocity=0.0, torque=cmd_trq)]
            )
            data[1]["time"].append(elapsed)
            data[1]["position"].append(pos)
            data[1]["velocity"].append(vel)
            data[1]["torque"].append(cmd_trq)
            data[1]["pos_ref"].append(pos_ref)
            data[1]["vel_ref"].append(vel_ref)
            data[1]["pos_pred"].append(xhat[0,0])
            data[1]["vel_pred"].append(xhat[1,0])
        else:
            failure_count += 1
        elapsed_end = time.time() - start_time
        cycle_time = elapsed_end - elapsed

        time.sleep(0.0095 - cycle_time)

control = threading.Thread(target=control_thread)
control.start() 

# plot trajectory
def plot_trajectory(xf, T, num_points=100):
    times = np.linspace(0, T, num_points)
    positions = []
    velocities = []
    
    for t in times:
        pos, vel = quintic_trajectory(xf, T, t)
        positions.append(pos)
        velocities.append(vel)
    
    plt.figure(figsize=(10, 5))
    
    plt.subplot(1, 2, 1)
    plt.plot(times, positions, label='Position')
    plt.xlabel('Time')
    plt.ylabel('Position')
    plt.title('Position vs Time')
    plt.legend()
    
    plt.subplot(1, 2, 2)
    plt.plot(times, velocities, label='Velocity', color='r')
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.title('Velocity vs Time')
    plt.legend()
    
    plt.tight_layout()
    plt.show()
# plot motor torque , velocitu, time

def plot_motor_data():
    print("length of data[1][time] : ", len(data[1]["time"]))
    # Create vector of times between each sample and print
    time_vector = np.diff(data[1]["time"])
    mean_time_vector = np.mean(time_vector)
    stdev_time_vector = np.std(time_vector)
    print("Mean time vector : ", mean_time_vector*1000, " ms")
    print("Standard deviation time vector : ", stdev_time_vector*1000, " ms")
    print("Last value of time vector : ", data[1]["time"][-1])
    print("This is the feailure countt : ", failure_count)
    plt.figure(figsize=(12, 8))
    # positoin plot
    plt.subplot(3, 1, 1)
    plt.scatter(data[1]["time"], data[1]["position"], label="Position Raw", color='blue', s=5)
    plt.plot(data[1]["time"], data[1]["position"], label="Position", color='blue')
    plt.plot(data[1]["time"], data[1]["pos_ref"], label="Position Ref", color='black')
    plt.scatter(data[1]["time"], data[1]["pos_pred"], label="Position Pred Raw", color='red', s=5)
    plt.plot(data[1]["time"], data[1]["pos_pred"], label="Position Pred", color='red')

    # velocity plot
    plt.subplot(3, 1, 2)
    plt.scatter(data[1]["time"], data[1]["velocity"], label="Velocity Raw", color='blue', s=5)
    plt.plot(data[1]["time"], data[1]["velocity"], label="Velocity", color='blue')
    plt.plot(data[1]["time"], data[1]["vel_ref"], label="Velocity Ref", color='black')
    plt.scatter(data[1]["time"], data[1]["vel_pred"], label="Velocity Pred Raw", color='red', s=5)
    plt.plot(data[1]["time"], data[1]["vel_pred"], label="Velocity Pred", color='red')
    
    plt.subplot(3, 1, 3)
    torque_bound_pos = np.ones(len(data[1]["time"])) * 0.014
    torque_bound_neg = np.ones(len(data[1]["time"])) * -0.014
    plt.scatter(data[1]["time"], data[1]["torque"], label="Torque", color='red', s=5)
    plt.step(data[1]["time"], data[1]["torque"], label="Torque", color='green')
    plt.plot(data[1]["time"], torque_bound_pos, label="Torque Bound Pos", linestyle='--', color='black')
    plt.plot(data[1]["time"], torque_bound_neg, label="Torque Bound Neg", linestyle='--', color='black')

    # plt.plot(data[1]["time"], data[1]["raw_torque"], label="Command Raw Torque", color='purple')
    plt.title("Motor Torque")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.grid()
    plt.legend()

    
    # plt.subplot(3, 1, 1)
    # plt.scatter(data[1]["time"], data[1]["position"], label="Position Raw", color='blue', s=5)
    # plt.plot(data[1]["time"], data[1]["position"], label="Position")
    # plt.plot(data[1]["time"], data[1]["pos_ref"], label="Position Ref", linestyle='--')
    # plt.title("Motor Position")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Position (rad)")
    # plt.grid()
    # plt.legend()

    # plt.subplot(3, 1, 2)
    # var = np.mean(np.square(data[1]["velocity"]))
    # print(f"\n\nRMSE ERROR NOISE : ", var)
    # plt.scatter(data[1]["time"], data[1]["velocity"], label="Velocity Raw", color='blue', s=5)
    # plt.plot(data[1]["time"], data[1]["velocity"], label="Velocity", color='orange')
    # plt.plot(data[1]["time"], data[1]["vel_ref"], label="Velocity Ref", linestyle='--')
    # plt.title("Motor Velocity")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Velocity (rad/s)")
    # plt.grid()
    # plt.legend()

   

    plt.tight_layout()
    s = f"K[0] = {K[0]}, K[1] = {K[1]}"
    plt.gcf().text(0.01, 0.97, s, fontsize=12, ha='left')
    plt.show()


# Call the plot function after the control thread has finished
try:
    control.join()
except KeyboardInterrupt:
    # supervisor.disable(1)
    # supervisor.disable(7)
    # supervisor.disable(3)
    plot_motor_data()
finally:
    plot_motor_data()
    print("Exiting control loop.")

    # Cleanup
    # os.system('clear')
