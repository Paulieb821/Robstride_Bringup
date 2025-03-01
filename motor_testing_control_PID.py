# motor_testing_control.py script
import math
import time
import threading
import matplotlib.pyplot as plt
import numpy as np
from joint_control_utils.PID_Controller import PID_Controller
from joint_control_utils.Joint_Trajectories import cubic_trajectory, quintic_trajectory
from actuator import RobstrideActuator, RobstrideActuatorConfig, RobstrideActuatorCommand, RobstrideConfigureRequest
from plot import Plot

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

# Select active motor
active_id = 3

# Get initial values
initials_gotten = False
while not initials_gotten:
    state = supervisor.get_actuators_state([active_id])
    if state:
        initial_pos = math.radians(state[0].position)
        initial_vel = math.radians(state[0].velocity)
        initials_gotten = True

# Initialize controller
T = 0.01
controller = PID_Controller(timestep=T, inertia=0.006556, bandwidth=10,Kd_correction=2.0, 
                            anti_windup_trq=6, pos_deadzone=0.1, vel_deadzone=0.1)


# Trajectorty Parameters
setpoint = -2
move_time = 2

# Motors and Logging
motor_ids = [1, 7, 3]
data = {motor_id: {"time": [], "position": [], "velocity": [], "torque": [], "pos_ref":[], "vel_ref":[], "vel_pred":[], "pos_pred":[]} for motor_id in motor_ids}

# Misc
max_time = move_time + 3
failure_count = 0

# Main Control Loop
def control_thread():
    # Get program start time
    start_time = time.time()
    # Control loop
    while True:
        # Elapsed time
        elapsed = time.time() - start_time
        # Check for shutdown
        if (elapsed > max_time):
            supervisor.command_actuators(
                [RobstrideActuatorCommand(actuator_id=active_id, position=0.0, velocity=0.0, torque=0)])
            supervisor.disable(1)
            supervisor.disable(7)
            supervisor.disable(3)
            break
        # Get trajectory references
        pos_ref, vel_ref, acc_ref = quintic_trajectory(initial_pos, setpoint, move_time, elapsed)
        # Get motor state
        state = supervisor.get_actuators_state([active_id])
        if state:
            # Process position and velocity
            pos = math.radians(state[0].position) - initial_pos
            vel = math.radians(state[0].velocity)
            # Calculate torque using PID controller
            cmd_trq = controller.update_controller(pos, pos_ref, vel_ref, acc_ref)
            # Send out command
            supervisor.command_actuators(
                [RobstrideActuatorCommand(actuator_id=active_id, position=0, velocity=0.0, torque=cmd_trq)]
            )
            # Data Logging
            data[1]["time"].append(elapsed)
            data[1]["position"].append(pos)
            data[1]["velocity"].append(vel)
            data[1]["torque"].append(cmd_trq)
            data[1]["pos_ref"].append(pos_ref)
            data[1]["vel_ref"].append(vel_ref)
            data[1]["pos_pred"].append(controller.xhat[0,0])
            data[1]["vel_pred"].append(controller.xhat[1,0])
        else:
            failure_count += 1

        # Ensure consistent frame times
        elapsed_end = time.time() - start_time
        cycle_time = elapsed_end - elapsed
        time.sleep(T - cycle_time)

# Start control thread
control = threading.Thread(target=control_thread)
control.start() 

# Call the plot function after the control thread has finished
try:
    control.join()
except KeyboardInterrupt:
    supervisor.disable(1)
    supervisor.disable(7)
    supervisor.disable(3)
finally:
    plotter = Plot(data)
    plotter.plot_motor_data()
    print("Exiting control loop.")