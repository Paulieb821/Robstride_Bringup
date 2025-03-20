import mujoco as mj
import os
import numpy as np
import threading
import time

from actuator import RobstrideActuator, RobstrideActuatorConfig, RobstrideActuatorCommand, RobstrideConfigureRequest
from robot_control_utils.TrajectoryPlanner import TrajectoryPlanner6dof as trajectory_planner

# Configuration
urdf_path = 'robot_models/4dof_arm_v2/4dof_arm_v2.xml'
site_name = 'endeff'
command_rate = 50

# Create supervisor
supervisor = RobstrideActuator(ports=['/dev/ttyUSB0'], py_actuators_config=[
    (1, RobstrideActuatorConfig(1)),    # J1
    (2, RobstrideActuatorConfig(3)),    # J2
    (3, RobstrideActuatorConfig(2)),    # J3
    (4, RobstrideActuatorConfig(1)),    # J4 - figure out how to account for belt drive
    ])
supervisor.run_main_loop(1)

# Startup sequence
print("Startup sequence")
time.sleep(1)
supervisor.enable(1)
supervisor.enable(2)
supervisor.enable(3)
supervisor.enable(4)

supervisor.configure_actuator(RobstrideConfigureRequest(actuator_id=1, torque_enabled=True, kp=20.0, kd=3.0, max_torque=6.0, zero_position=False))
time.sleep(0.5)
supervisor.configure_actuator(RobstrideConfigureRequest(actuator_id=2, torque_enabled=True, kp=20.0, kd=3.0, max_torque=6.0, zero_position=False))
time.sleep(0.5)
supervisor.configure_actuator(RobstrideConfigureRequest(actuator_id=3, torque_enabled=True, kp=20.0, kd=3.0, max_torque=6.0, zero_position=False))
time.sleep(0.5)
supervisor.configure_actuator(RobstrideConfigureRequest(actuator_id=4, torque_enabled=True, kp=20.0, kd=3.0, max_torque=6.0, zero_position=False))
time.sleep(0.5)

# Load the MuJoCo model and data
model = mj.MjModel.from_xml_path(urdf_path)
data = mj.MjData(model)
site = data.site(site_name)

# Create trajectory
traj = trajectory_planner(model, data, site, [0.0, 0.0, 0.0, 0.0], command_rate)
traj.addHold(5)
traj.addLinearMove_3dof(np.array([0, 0.4, 0.1]), 1)
traj.addLinearMove_3dof(np.array([0, 0.7, 0.1]), 1)
traj.addLinearMove_3dof(np.array([0.5, 0.5, 0.2]), 1)
traj.addLinearMove_3dof(np.array([-0.5, 0.5, 0.2]), 2)
traj.addLinearMove_3dof(np.array([0, 0.5, 0.4]), 1)
traj.addLinearMove_3dof(np.array([0, 0.2, 0.8]), 1)
# Adjust for gear ratio
traj.pos_ref[:,3] = traj.pos_ref[:,3] * 3
traj.vel_ref[:,3] = traj.vel_ref[:,3] * 3
# Send out commands
start_time = time.time()

def control_thread():
    while True:
        elapsed = time.time() - start_time
        step = min(int(np.floor(elapsed*command_rate)), np.shape(traj.jointSpaceTraj)[0]-1)
        supervisor.command_actuators([
            RobstrideActuatorCommand(actuator_id=1, position=np.rad2deg(traj.pos_ref[step, 0]), velocity=np.rad2deg(traj.vel_ref[step, 0]), torque=0),
            RobstrideActuatorCommand(actuator_id=2, position=-np.rad2deg(traj.pos_ref[step, 1]), velocity=-np.rad2deg(traj.vel_ref[step, 1]), torque=0),
            RobstrideActuatorCommand(actuator_id=3, position=np.rad2deg(traj.pos_ref[step, 2]), velocity=np.rad2deg(traj.vel_ref[step, 2]), torque=0),
            RobstrideActuatorCommand(actuator_id=4, position=-np.rad2deg(traj.pos_ref[step, 3]), velocity=-np.rad2deg(traj.vel_ref[step, 3]), torque=0)
        ])
        # print(traj.pos_ref[step, 3])
        time.sleep(1/command_rate)
        
    
control = threading.Thread(target=control_thread)
control.start()

# Call the plot function after the control thread has finished
try:
    control.join()
except KeyboardInterrupt:
    supervisor.disable(1)
    supervisor.disable(2)
    supervisor.disable(3)
    supervisor.disable(4)
finally:
    #plotter = Plot(data)
    #plotter.plot_motor_data()
    print("Exiting control loop.")