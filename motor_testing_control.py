# motor_testing_control.py script
import math
import time
import threading
import matplotlib.pyplot as plt
import numpy as np
from joint_control_utils.PID_Controller import PID_Controller
from joint_control_utils.PD_Controller import PD_Controller
from joint_control_utils.Joint_Trajectories import cubic_trajectory, quintic_trajectory
from actuator import RobstrideActuator, RobstrideActuatorConfig, RobstrideActuatorCommand, RobstrideConfigureRequest
from plot import Plot

# Create Supervisor
supervisor = RobstrideActuator(ports=['/dev/ttyUSB0'], py_actuators_config=[
    (1, RobstrideActuatorConfig(1)),    # J1
    (2, RobstrideActuatorConfig(3)),    # J2
    (3, RobstrideActuatorConfig(1)),    # J3
    ])
supervisor.run_main_loop(1)

# Startup sequence
print("Startup sequence")
time.sleep(1)
supervisor.enable(1)
supervisor.enable(2)
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

# System parameters - RS01 on 2.5kg pendulum arm
# Bandwidth = 30.0, no deadzones
# rotor_inertia = 0.01
# added_inertia = (1.2 * 0.3**2 + 0.224 * 0.3**2/3)
# total_inertia = rotor_inertia + added_inertia
# comp_trq = 1.2 * 0.3 * 9.81 * (1.1 + 0.224/2)
# gear_ratio = 1
# relative_pos = True

# System parameters - Elbow
# Bandwidth = 30.0, no deadzones
rotor_inertia = 0.3
added_inertia = 0.0
total_inertia = rotor_inertia + added_inertia
comp_trq = 0.0
gear_ratio = 3
relative_pos = False

# System parameters - J1 spinning J2
# Bandwidth = 35.0, vel_deadzone = 0.1, everything else default
# rotor_inertia = 0.01
# added_inertia = 0.011729
# total_inertia = rotor_inertia + added_inertia
# comp_trq = 0.0
# gear_ratio = 1
# relative_pos = False

# System parameters - J2 in bracket
# Bandwidth = 30.0, everything else default
# rotor_inertia = 0.1
# added_inertia = 0.0
# total_inertia = rotor_inertia + added_inertia
# comp_trq = 0.3 * 9.81 * (1.1 * 0.75  + 0.224 * 0.5)
# gear_ratio = 1
# relative_pos = True

# Initialize controller
T = 0.01
if relative_pos:
    ip = 0.0
else:
    ip = initial_pos
controller = PD_Controller(timestep=T, gear_ratio=gear_ratio, inertia=total_inertia, bandwidth=30.0, Kd_correction=1.0, 
                            anti_windup_trq=6.0, pos_deadzone=0.1, vel_deadzone=0.0,
                            initial_pos=ip, initial_vel=initial_vel, joint_limits=[3, -3])


# Trajectorty Parameters
setpoint = 0.8
move_time = 1

# Motors and Logging
motor_ids = [1, 2, 3]
data = {motor_id: {"time": [], "position": [], "velocity": [], "torque": [], "pos_ref":[], "vel_ref":[], "vel_pred":[], "pos_pred":[]} for motor_id in motor_ids}

# Misc
max_time = move_time + 3

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
        if relative_pos:
            pos_ref, vel_ref, acc_ref = quintic_trajectory(0.0, setpoint, move_time, elapsed)
        else:
            pos_ref, vel_ref, acc_ref = quintic_trajectory(initial_pos/gear_ratio, setpoint, move_time, elapsed)
        # Get motor state
        state = supervisor.get_actuators_state([active_id])
        if state:
            # Process position and velocity
            if relative_pos:
                pos = math.radians(state[0].position) - initial_pos
            else:
                pos = math.radians(state[0].position)
            vel = math.radians(state[0].velocity)
            # Calculate torque using PID controller
            cmd_trq = controller.update_controller(pos, vel, pos_ref, vel_ref, acc_ref, comp_trq)
            # Send out command
            supervisor.command_actuators(
                [RobstrideActuatorCommand(actuator_id=active_id, position=0, velocity=0.0, torque=cmd_trq)]
            )
            # Data Logging
            data[active_id]["time"].append(elapsed)
            data[active_id]["position"].append(pos/gear_ratio)
            data[active_id]["velocity"].append(vel/gear_ratio)
            data[active_id]["torque"].append(cmd_trq)
            data[active_id]["pos_ref"].append(pos_ref)
            data[active_id]["vel_ref"].append(vel_ref)
            data[active_id]["pos_pred"].append(controller.xhat[0,0])
            data[active_id]["vel_pred"].append(controller.xhat[1,0])
        else: 
            print("Didn't get state")

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