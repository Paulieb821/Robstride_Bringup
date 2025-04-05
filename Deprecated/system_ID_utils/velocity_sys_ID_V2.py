# motor_testing_control.py script
import argparse
import math
import time
import os
import threading
import matplotlib.pyplot as plt
import numpy as np
import control as ctrl
from sklearn.linear_model import LinearRegression
 
from actuator import RobstrideActuator, RobstrideActuatorConfig, RobstrideActuatorCommand, RobstrideConfigureRequest

# Motor ID
motor_id = 1

# Runtime
max_time = 3
torque_amp = 0.15
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
        state = supervisor.get_actuators_state([motor_id])
        if state:
            # Process position and velocity
            pos = math.radians(state[0].position)
            vel = math.radians(state[0].velocity)
            torque_temp = math.radians(state[0].torque)
            
            # Torque Input
            cmd_trq = torque_amp
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


def sys_id(vel, time):
    # Format
    y = torque_amp * time * 3
    x = vel/3

    y = y.reshape(-1,1)
    x = x.reshape(-1,1)

    # Initialize the linear regression model
    model = LinearRegression()

    # Fit the model to the data
    model.fit(x, y)

    # Get the coefficient (I) and the intercept
    I = model.coef_[0]

    print("\nSystem Inertia: ", I)
    

def plot_motor_data():
    pos_data = np.array(data["position"][20:-1])
    torque_data = np.array(data["torque"][20:-1])
    time_data = np.array(data["time"][20:-1])
    velocity_data = np.array(data["velocity"][20:-1])

    sys_id(velocity_data, time_data)
    # Create vector of times between each sample and print
    time_vector = np.diff(data["time"])
    mean_time_vector = np.mean(time_vector)
    stdev_time_vector = np.std(time_vector)
    plt.figure(figsize=(12, 8))
    
    plt.subplot(3, 1, 1)
    for angle in data["position"]:
        if angle < -math.pi and angle > -0.1*math.pi:
            angle += 2 * math.pi
    plt.scatter(time_data, pos_data, label="Position Raw", color='blue', s=5)
    plt.plot(time_data, pos_data, label="Position")
    plt.title("Motor Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.grid()
    plt.legend()

    plt.subplot(3, 1, 2)
    var = np.mean(np.square(data["velocity"]))
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
