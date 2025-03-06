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

# Select active motor
active_ids = [1,2]

# Get initial values
initials_gotten = False
while not initials_gotten:
    state = supervisor.get_actuators_state(active_ids)
    if state and len(state) == 2:
        initial_pos_1 = math.radians(state[0].position)
        initial_vel_1 = math.radians(state[0].velocity)
        initial_pos_2 = math.radians(state[1].position)
        initial_vel_2 = math.radians(state[1].velocity)
        initials_gotten = True
        print("Motor enabled")

# System parameters - J1 spinning J2, no J3 attached
# J1 - Bandwidth = 35.0, vel_deadzone = 0.1, everything else default
rotor_inertia_1 = 0.01
added_inertia_1 = 0.011729
total_inertia_1 = rotor_inertia_1 + added_inertia_1
comp_trq_1 = 0.0
gear_ratio_1 = 1
relative_pos_1 = True
# Bandwidth = 30.0, everything else default
rotor_inertia_2 = 0.1
added_inertia_2 = 0.0
total_inertia_2 = rotor_inertia_2 + added_inertia_2
comp_trq_2 = 0.0
gear_ratio_2 = 1
relative_pos_2 = True

# Initialize joint controllers
T = 0.01
# J1
if relative_pos_1:
    ip = 0.0
else:
    ip = initial_pos_1
controller_1 = PD_Controller(timestep=T, gear_ratio=gear_ratio_1, inertia=total_inertia_1, bandwidth=10.0, Kd_correction=1.0, 
                            anti_windup_trq=6.0, pos_deadzone=0.005, vel_deadzone=0.1,
                            initial_pos=ip, initial_vel=initial_vel_1, joint_limits=[-3, -3])
# J2
if relative_pos_2:
    ip = 0.0
else:
    ip = initial_pos_2
controller_2 = PD_Controller(timestep=T, gear_ratio=gear_ratio_2, inertia=total_inertia_2, bandwidth=15.0, Kd_correction=1.0, 
                            anti_windup_trq=6.0, pos_deadzone=0.1, vel_deadzone=0.0,
                            initial_pos=ip, initial_vel=initial_vel_2, joint_limits=[3, -3])


# Trajectorty Parameters
setpoint_1 = 1.5
setpoint_2 = 0.001
move_time = 0.5

# Motors and Logging
motor_ids = [1, 2, 3]
data = {motor_id: {"time": [], "position": [], "velocity": [], "torque": [], "pos_ref":[], "vel_ref":[], "vel_pred":[], "pos_pred":[]} for motor_id in motor_ids}

# Misc
max_time = move_time + 15

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
                [RobstrideActuatorCommand(actuator_id=active_ids[0], position=0.0, velocity=0.0, torque=0.0),
                 RobstrideActuatorCommand(actuator_id=active_ids[1], position=0.0, velocity=0.0, torque=0)])
            supervisor.disable(1)
            supervisor.disable(2)
            supervisor.disable(3)
            break
        # Get trajectory references
        if relative_pos_1:
            pos_ref_1, vel_ref_1, acc_ref_1 = quintic_trajectory(0.0, setpoint_1, move_time, elapsed)
        else:
            pos_ref_1, vel_ref_1, acc_ref_1 = quintic_trajectory(initial_pos_1/gear_ratio_1, setpoint_1, move_time, elapsed)
        if relative_pos_2:
            pos_ref_2, vel_ref_2, acc_ref_2 = quintic_trajectory(0.0, setpoint_2, move_time, elapsed)
        else:
            pos_ref_2, vel_ref_2, acc_ref_2 = quintic_trajectory(initial_pos_2/gear_ratio_2, setpoint_2, move_time, elapsed)
        # Get motor state
        state = supervisor.get_actuators_state(active_ids)
        if state:
            # Process position and velocity
            if relative_pos_1:
                pos_1 = math.radians(state[0].position) - initial_pos_1
            else:
                pos_1 = math.radians(state[0].position)
            vel_1 = math.radians(state[0].velocity)
            if relative_pos_2:
                pos_2 = math.radians(state[1].position) - initial_pos_2
            else:
                pos_2 = math.radians(state[1].position)
            vel_2 = math.radians(state[1].velocity)
            # Calculate torque using PID controller
            cmd_trq_1 = controller_1.update_controller(pos_1, vel_1, pos_ref_1, vel_ref_1, acc_ref_1, comp_trq_1)
            cmd_trq_2 = controller_2.update_controller(pos_2, vel_2, pos_ref_2, vel_ref_2, acc_ref_2, comp_trq_2)
            # Send out command
            supervisor.command_actuators(
                [RobstrideActuatorCommand(actuator_id=active_ids[0], position=0.0, velocity=0.0, torque=cmd_trq_1),
                RobstrideActuatorCommand(actuator_id=active_ids[1], position=0.0, velocity=0.0, torque=cmd_trq_2)]
            )
            # Data Logging
            data[active_ids[0]]["time"].append(elapsed)
            data[active_ids[0]]["position"].append(pos_1/gear_ratio_1)
            data[active_ids[0]]["velocity"].append(vel_1/gear_ratio_1)
            data[active_ids[0]]["torque"].append(cmd_trq_1)
            data[active_ids[0]]["pos_ref"].append(pos_ref_1)
            data[active_ids[0]]["vel_ref"].append(vel_ref_1)
            data[active_ids[0]]["pos_pred"].append(controller_1.xhat[0,0])
            data[active_ids[0]]["vel_pred"].append(controller_1.xhat[1,0])
            data[active_ids[1]]["time"].append(elapsed)
            data[active_ids[1]]["position"].append(pos_2/gear_ratio_2)
            data[active_ids[1]]["velocity"].append(vel_2/gear_ratio_2)
            data[active_ids[1]]["torque"].append(cmd_trq_2)
            data[active_ids[1]]["pos_ref"].append(pos_ref_2)
            data[active_ids[1]]["vel_ref"].append(vel_ref_2)
            data[active_ids[1]]["pos_pred"].append(controller_2.xhat[0,0])
            data[active_ids[1]]["vel_pred"].append(controller_2.xhat[1,0])

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