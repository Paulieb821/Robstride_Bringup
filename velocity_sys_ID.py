# motor_testing_control.py script
import argparse
import math
import time
import os
import threading
import matplotlib.pyplot as plt
import numpy as np
import control as ctrl
 
from actuator import RobstrideActuator, RobstrideActuatorConfig, RobstrideActuatorCommand, RobstrideConfigureRequest

# Motor ID
motor_id = 1

# Runtime
max_time = 20
torque_amp = 0.05
# Create Supervisor
supervisor = RobstrideActuator(ports=['/dev/ttyUSB0'], py_actuators_config=[
    (motor_id, RobstrideActuatorConfig(1)),    # J1
    ])

supervisor.run_main_loop(1)

# Startup sequence
print("Startup sequence")
time.sleep(1)
supervisor.enable(motor_id)
print("Motor enabled")

initials_gotten = False
while not initials_gotten:
    state = supervisor.get_actuators_state([motor_id])
    if state:
        initial_pos = math.radians(state[0].position)
        initials_gotten = True

# Logging
data = {"time": [], "position": [], "velocity": [], "torque": [], "torque_meas": []}
failure_count = 0

def control_thread():
    start_time = time.time()
    
    while True:
        # Get elapsed time
        elapsed = time.time() - start_time
        if (elapsed > max_time):
            supervisor.command_actuators(
                [RobstrideActuatorCommand(actuator_id=motor_id, position=0.0, velocity=0.0, torque=0)])
            supervisor.disable(1)
            supervisor.disable(7)
            supervisor.disable(3)
            break
        # Get motor state
        state = supervisor.get_actuators_state([1])
        if state:
            # Process position and velocity
            pos = math.radians(state[0].position)
            vel = math.radians(state[0].velocity)
            torque_temp = math.radians(state[0].torque)
            
            # Torque Input
            omega = 5
            cmd_trq = 0.1 * np.sin(omega * elapsed)
            #cmd_trq = -torque_amp
            print(round(pos,2), round(vel,2), round(cmd_trq,2))
            # Send out command
            supervisor.command_actuators(
                [RobstrideActuatorCommand(actuator_id=motor_id, position=0, velocity=0.0, torque=cmd_trq)]
            )
            data["time"].append(elapsed)
            data["position"].append(pos)
            data["velocity"].append(vel)
            data["torque"].append(cmd_trq)
            data["torque_meas"].append(torque_temp)

            
        else:
            failure_count += 1
        
        time.sleep(0.01)

control = threading.Thread(target=control_thread)
control.start() 


def sys_id(vel, torque, time, bandwidth):
    # Setup
    y = vel
    u = torque

    # Lambdas
    L0 = bandwidth**2
    L1 = 2*bandwidth

    # Solve for necessary signals
    filtered_y = np.zeros((2, len(y)))
    filtered_u = np.zeros((2, len(u)))
    diff_eq_mat = np.array([[0, 1], [-L0, -L1]])
    diff_eq_vec = np.array([[0], [L0]])
    filtered_y[:,0] = np.array([y[0], 0])
    for i in range(len(time)-1):
        dt = time[i+1] - time[i]
        filtered_y[:,i+1] = np.reshape(np.reshape(filtered_y[:,i], (-1, 1)) + (diff_eq_mat @ np.reshape(filtered_y[:,i], (-1, 1)) + diff_eq_vec * y[i]) * dt, -1)
        filtered_u[:,i+1] = np.reshape(np.reshape(filtered_u[:,i], (-1, 1)) + (diff_eq_mat @ np.reshape(filtered_u[:,i], (-1, 1)) + diff_eq_vec * u[i]) * dt, -1)

    # Set up vectors
    cursive_y = filtered_y[1,:]
    cursive_u = np.vstack((filtered_u[0,:], -filtered_y[0,:]))
    
    # Solve for system
    U_matrix = cursive_u @ cursive_u.T
    Y_matrix = cursive_u @ cursive_y.T
    cursive_P = np.linalg.pinv(U_matrix) @ Y_matrix

    # Get system parameters
    b0 = cursive_P[0]
    a0 = cursive_P[1]

    J = 1/b0
    c = a0/b0

    # Print system parameters
    print("\nSystem parameters")
    print("b0: ", round(b0, 7))
    print("a0: ", round(a0, 7))
    print("\nTransfer function: ")
    print(f"G(s) = {round(b0,2)})/(s + {round(a0,2)})\n")

    print("Motor Parameters")
    print("J: ", round(J, 7))
    print("c: ", round(c, 7), "\n")

    G = ctrl.TransferFunction([b0 * torque_amp], [1, a0])
    # Compute step response over the same time horizon
    time = np.linspace(0, max(time), 100)
    t_out, y_out = ctrl.step_response(G, T=time)
    return t_out, y_out
    

def plot_motor_data():
    pos_data = np.array(data["position"][100:-1])
    torque_data = np.array(data["torque"][100:-1])
    time_data = np.array(data["time"][100:-1])
    velocity_data = np.array(data["velocity"][100:-1])

    t_out, y_out = sys_id(velocity_data, torque_data, time_data, 0.1)
    print("length of data[time] : ", len(data["time"]))
    print("length of data[position] : ", len(data["position"]))
    print("length of data[velocity] : ", len(data["velocity"]))
    print("length of data[torque] : ", len(data["torque"]))
    # Create vector of times between each sample and print
    time_vector = np.diff(data["time"])
    mean_time_vector = np.mean(time_vector)
    stdev_time_vector = np.std(time_vector)
    print("Mean time vector : ", mean_time_vector*1000, " ms")
    print("Standard deviation time vector : ", stdev_time_vector*1000, " ms")
    print("Last value of time vector : ", data["time"][-1])
    print("This is the feailure countt : ", failure_count)
    plt.figure(figsize=(12, 8))
    
    plt.subplot(3, 1, 1)
    for angle in data["position"]:
        if angle < -math.pi and angle > -0.1*math.pi:
            angle += 2 * math.pi
    plt.scatter(time_data, pos_data, label="Position Raw", color='blue', s=5)
    plt.plot(time_data, pos_data, label="Position")
    plt.plot(t_out, y_out, label="Step Response")
    plt.title("Motor Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.grid()
    plt.legend()

    plt.subplot(3, 1, 2)
    var = np.mean(np.square(data["velocity"]))
    print(f"\n\nRMSE ERROR NOISE : ", var)
    plt.scatter(time_data, velocity_data, label="Velocity Raw", color='blue', s=5)
    plt.plot(time_data, velocity_data, label="Velocity", color='orange')
    plt.title("Motor Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.grid()
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.scatter(time_data, torque_data, label="Torque", color='red', s=10)
    plt.plot(time_data, torque_data, label="Torque", color='green')
    # plt.plot(data["time"], data["torque_meas"], label="Torque Meas", linestyle='--')
    # plt.title("Motor Torque")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.grid()
    plt.legend()

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
