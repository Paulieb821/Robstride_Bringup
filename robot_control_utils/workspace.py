import numpy as np
import mujoco as mj
import NRIK

# This file is used to calculate the workspace of a 4 dof robot arm based on the mjcf and checks whether the input position is within the workspace

urdf_path = 'robot_models/Sim_Arm_4DOF_Mar_25/robot.xml'
site_name = 'endeff'

# Load the MuJoCo model and data
model = mj.MjModel.from_xml_path(urdf_path)
data = mj.MjData(model)
site = data.site(site_name)
ik = NRIK(model, data, site)

jnt_min = model.jnt_range[:, 0]
jnt_max = model.jnt_range[:, 2]

max_reach = 0.6 # outer position
min_reach = 0.1 # inner most position

def within_workspace(position, tol=1e-3):
    p = np.asarray(position, dtype=float)

    # initial spherical shell check (very rough reach values need tuning)
    r = np.linalg.norm(p)
    if r > max_reach - tol or r < min_reach + tol:
        print("Max/Min Reach exceeded")
        return False
    
    # IK check
    q = ik.solveIK_3dof(p)
    if q is None or np.any(np.isnan(q)):
        print("IK failed")
        return False
    
    # joint range check
    if np.any(q < jnt_min + tol) or np.any(q > jnt_max - tol):
        print("Joint range exceeded")
        return False
    
    # All checks passed
    return True
