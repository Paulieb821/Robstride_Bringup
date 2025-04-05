import mujoco as mj
import can
import robstride
import time
import numpy as np

from robot_control_utils.TrajectoryPlanner import TrajectoryPlanner6dof as trajectory_planner
from robot_control_utils.NRIK import NRIK as ik

# Mujoco configuration
urdf_path = 'robot_models/Sim_Arm_J1_Broken/robot.xml'
site_name = 'endeff'
command_rate = 50

# Load the MuJoCo model and data
model = mj.MjModel.from_xml_path(urdf_path)
data = mj.MjData(model)
site = data.site(site_name)

# Get initial position of arm
endeff_pos = data.site.xpos
print(endeff_pos)

# Run Trajectory
with can.Bus() as bus:
    # Declare client object
    rs_client = robstride.Client(bus)

    # Configure and Enable Motors
    rs_client.write_param(1, 'run_mode', robstride.RunMode.Position)
    rs_client.write_param(2, 'run_mode', robstride.RunMode.Position)
    rs_client.write_param(3, 'run_mode', robstride.RunMode.Position)
    rs_client.write_param(4, 'run_mode', robstride.RunMode.Position)

    # Then enable the motor
    rs_client.enable(1)
    rs_client.enable(2)
    rs_client.enable(3)
    rs_client.enable(4)

    # Send out commands
    start_time = time.time()

    while True:
            elapsed = time.time() - start_time
            step = min(int(np.floor(elapsed*command_rate)), np.shape(traj.jointSpaceTraj)[0]-1)
            rs_client.write_param(1, 'loc_ref', traj.pos_ref[step, 0])
            rs_client.write_param(2, 'loc_ref', traj.pos_ref[step, 1])
            rs_client.write_param(3, 'loc_ref', traj.pos_ref[step, 2])
            rs_client.write_param(4, 'loc_ref', traj.pos_ref[step, 3])

# Call the plot function after the control thread has finished
try:
    control.join()
except KeyboardInterrupt:
    rs_client.disable(1)
    rs_client.disable(2)
    rs_client.disable(3)
    rs_client.disable(4)
finally:
    #plotter = Plot(data)
    #plotter.plot_motor_data()
    print("Exiting control loop.")