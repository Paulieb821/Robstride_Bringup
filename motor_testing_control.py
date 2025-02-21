# motor_testing_control.py script
import argparse
import math
import time
import os
import threading
import matplotlib.pyplot as plt
import numpy as np
 
from actuator import RobstrideActuator, RobstrideActuatorConfig, RobstrideActuatorCommand, RobstrideConfigureRequest
# Cleanup
#os.system('clear')

# Create Supervisor
supervisor = RobstrideActuator(ports=['/dev/ttyUSB0'], py_actuators_config=[
    (1, RobstrideActuatorConfig(1)),    # J1
    (7, RobstrideActuatorConfig(3)),    # J2
    (3, RobstrideActuatorConfig(1)),    # J3
    ])

supervisor.run_main_loop(1)

print("Startup sequence")
time.sleep(1)
supervisor.enable(1)
supervisor.enable(7)
supervisor.enable(3)
print("Motor enabled")

# User Parameter
setpoint = 2
move_time = 1
deadzone_torque = 0.1
torque_limit = 0.6

# GainsS
kd = 0.4
kp = 0.6
motor_ids = [1, 7, 3]
data = {motor_id: {"time": [], "position": [], "velocity": [], "torque": [], "pos_ref":[], "vel_ref":[], "raw_torque":[]} for motor_id in motor_ids}

def trajectory(xf, T, time):
    if time < T:
        c2 = 3*xf/pow(T,2)
        c3 = -2*xf/pow(T,3)
        return (c2*pow(time,2) + c3*pow(time,3), 2*c2*time + 3*c3*pow(time,2))
    else:
        return (xf, 0)
max_time = move_time + 2
def control_thread():
    start_time = time.time()
    while True:
        # Get elapsed time
        elapsed = time.time() - start_time
        if (elapsed > max_time):
            supervisor.disable(1)
            supervisor.disable(7)
            supervisor.disable(3)
            break
        pos_ref, vel_ref = trajectory(setpoint, move_time, elapsed)
        # Get motor state
        state = supervisor.get_actuators_state([1])
        if state:
            # Process position and velocity
            pos = math.radians(state[0].position)
            vel = math.radians(state[0].velocity)
            # Calculate torque using PD controller
            cmd_trq = kp*(pos_ref-pos) + kd*(vel_ref-vel)
            data[1]["raw_torque"].append(cmd_trq)
            # Torque Limits
            # if abs(cmd_trq) < deadzone_torque:
            #     cmd_trq = 0
            # elif abs(cmd_trq) > torque_limit:
            #     cmd_trq = (cmd_trq/abs(cmd_trq))*torque_limit
            # Position Limits
            if abs(pos) > 3.1:
                cmd_trq = 0
            # if abs(vel) > 4:
            #     cmd_trq = 0
            # Logging
            #cmd_trq = 0
            print(round(pos,2), round(vel,2), round(cmd_trq,2))
            print(round(pos_ref,2), round(vel_ref,2))
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
        
        time.sleep(0.01)

control = threading.Thread(target=control_thread)
control.start() 

# plot trajectory
def plot_trajectory(xf, T, num_points=100):
    times = np.linspace(0, T, num_points)
    positions = []
    velocities = []
    
    for t in times:
        pos, vel = trajectory(xf, T, t)
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
    plt.figure(figsize=(12, 8))
    plt.subplot(3, 1, 1)
    plt.scatter(data[1]["time"], data[1]["position"], label="Position Raw", color='blue')
    plt.plot(data[1]["time"], data[1]["position"], label="Position")
    plt.plot(data[1]["time"], data[1]["pos_ref"], label="Position Ref", linestyle='--')
    plt.title("Motor Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.grid()
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.scatter(data[1]["time"], data[1]["velocity"], label="Velocity Raw", color='blue')
    plt.plot(data[1]["time"], data[1]["velocity"], label="Velocity", color='orange')
    plt.plot(data[1]["time"], data[1]["vel_ref"], label="Velocity Ref", linestyle='--')
    plt.title("Motor Velocity")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.grid()
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.scatter(data[1]["time"], data[1]["torque"], label="Torque", color='red')
    plt.plot(data[1]["time"], data[1]["torque"], label="Torque", color='green')
    plt.plot(data[1]["time"], data[1]["raw_torque"], label="Command Raw Torque", color='purple')
    plt.title("Motor Torque")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.show()


# Call the plot function after the control thread has finished
try:
    control.join()
except KeyboardInterrupt:
    supervisor.disable(1)
    supervisor.disable(7)
    supervisor.disable(3)
    plot_motor_data()
finally:
    plot_motor_data()
    print("Exiting control loop.")

    # Cleanup
    # os.system('clear')
