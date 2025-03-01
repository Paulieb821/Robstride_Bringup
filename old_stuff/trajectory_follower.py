import mujoco as mj
import os
import numpy as np
import threading
import time

from actuator import RobstrideActuator, RobstrideActuatorConfig, RobstrideActuatorCommand
from controller_utils.TrajectoryPlanner import TrajectoryPlanner6dof as trajectory_planner


# Configuration
urdf_path = 'robot_models/4dof_arm_v2/4dof_arm_v2.xml'
site_name = 'endeff'
command_rate = 50

# Create supervisor
supervisor = RobstrideActuator(ports=['/dev/ttyUSB0'], py_actuators_config=[
    (1, RobstrideActuatorConfig(1)),    # J1
    (2, RobstrideActuatorConfig(3)),    # J2
    (3, RobstrideActuatorConfig(1)),    # J3
    (4, RobstrideActuatorConfig(1)),    # J4 - figure out how to account for belt drive
    ])

supervisor.run_main_loop(1)

# Load the MuJoCo model and data
model = mj.MjModel.from_xml_path(urdf_path)
data = mj.MjData(model)
site = data.site(site_name)

# Create trajectory
traj = trajectory_planner(model, data, site, [0.0, 0.0, 0.0, 0.0], command_rate)
traj.addHold(5)
traj.addLinearMove_3dof(np.array([0.0, 0.3, 0.0]), 2)
traj.addLinearMove_3dof(np.array([0.3, 0.3, 0.0]), 2)
traj.addLinearMove_3dof(np.array([-0.3, 0.3, 0.0]), 2)
traj.addLinearMove_3dof(np.array([0.0, 0.3, 0.3]), 2)

# Send out commands
start_time = time.time()

def control_thread():
    while True:
        elapsed = time.time() - start_time
        step = min(int(np.floor(elapsed*command_rate)), np.shape(traj.jointSpaceTraj)[0]-1)
        supervisor.command_actuators(
            [RobstrideActuatorCommand(actuator_id=1, position=traj.jointSpaceTraj[step,0], velocity=0, torque=0),
            RobstrideActuatorCommand(actuator_id=2, position=traj.jointSpaceTraj[step,1], velocity=0, torque=0),
            RobstrideActuatorCommand(actuator_id=3, position=traj.jointSpaceTraj[step,2], velocity=0, torque=0),
            RobstrideActuatorCommand(actuator_id=4, position=traj.jointSpaceTraj[step,3], velocity=0, torque=0)],
        )
        time.sleep(1/command_rate)
        
    
control = threading.Thread(target=control_thread)
control.start()
