# motor_testing_control.py script
import math
import time
import threading
import matplotlib.pyplot as plt
import numpy as np
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
active_id = 2

# Get initial values
initials_gotten = False
while not initials_gotten:
    state = supervisor.get_actuators_state([active_id])
    if state:
        initial_pos = math.radians(state[0].position)
        initial_vel = math.radians(state[0].velocity)
        initials_gotten = True
        print("Motor enabled")

# Change motor operating mode
result = supervisor.configure_actuator(RobstrideConfigureRequest(actuator_id=active_id, torque_enabled=True, kp=60.0, kd=3.0, max_torque=6.0))
print(result)

# Motor Parameters
relative_pos = False
gear_ratio = 1
T = 0.02

# Trajectorty Parameters
setpoint = -1.57
move_time = 1

# Motors and Logging
motor_ids = [1, 2, 3]
data = {motor_id: {"time": [], "position": [], "velocity": [], "torque": [], "pos_ref":[], "vel_ref":[], "vel_pred":[], "pos_pred":[]} for motor_id in motor_ids}

# Misc
max_time = move_time + 20

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
            supervisor.command_actuators([RobstrideActuatorCommand(actuator_id=active_id, position=0.0, velocity=0.0, torque=0.0)])
            supervisor.disable(1)
            supervisor.disable(7)
            supervisor.disable(3)
            break
        # Get trajectory references
        if relative_pos:
            pos_ref, vel_ref, acc_ref = quintic_trajectory(0.0, setpoint, move_time, elapsed)
        else:
            pos_ref, vel_ref, acc_ref = quintic_trajectory(initial_pos/gear_ratio, setpoint, move_time, elapsed)
        # Read state
        state = supervisor.get_actuators_state([active_id])
        if state:
            # Process position and velocity
            if relative_pos:
                pos = math.radians(state[0].position) - initial_pos
            else:
                pos = math.radians(state[0].position)
            vel = math.radians(state[0].velocity)
            print(round(state[0].torque,2))
            print("Ref: ", round(pos_ref,2), "Real: ", round(pos,2))
            # Send position command
            comp_trq = 1.2 * 0.3 * 9.81 * (1.1 * 0.75  + 0.224 * 0.5) * np.sin(pos)
            supervisor.command_actuators(
                [RobstrideActuatorCommand(actuator_id=active_id, position=np.rad2deg(pos_ref), velocity=np.rad2deg(vel_ref), torque=comp_trq)]
            )
            # Data Logging
            data[active_id]["time"].append(elapsed)
            data[active_id]["position"].append(pos/gear_ratio)
            data[active_id]["velocity"].append(vel/gear_ratio)
            #data[active_id]["torque"].append(cmd_trq)
            data[active_id]["pos_ref"].append(pos_ref)
            data[active_id]["vel_ref"].append(vel_ref)
            #data[active_id]["pos_pred"].append(controller.xhat[0,0])
            #data[active_id]["vel_pred"].append(controller.xhat[1,0])
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
    #plotter = Plot(data)
    #plotter.plot_motor_data()
    print("Exiting control loop.")