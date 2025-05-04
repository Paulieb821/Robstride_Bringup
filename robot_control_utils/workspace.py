import numpy as np
import mujoco as mj
import NRIK

# This file is used to calculate the workspace of a 4 dof robot arm based on the mjcf and checks whether the input position is within the workspace

class Workspace:
    def __init__(self, urdf_path, model, data, ik, site_name="endeff", max_reach=0.6, min_reach=0.1):
        self.urdf_path = urdf_path
        self.site_name = site_name
        self.model = model
        self.data = data
        self.site = self.data.site(self.site_name)
        self.ik = ik
        self.jnt_min = self.model.jnt_range[:, 0]
        self.jnt_max = self.model.jnt_range[:, 1]
        self.max_reach = max_reach
        self.min_reach = min_reach


    def within_workspace(self, position, tol=1e-3):
        p = np.asarray(position, dtype=float)

        # initial spherical shell check (very rough reach values need tuning)
        r = np.linalg.norm(p)
        if r > self.max_reach - tol or r < self.min_reach + tol:
            print("Max/Min Reach exceeded")
            return False
        
        # IK check
        q = self.ik.solveIK_3dof(p)
        if q is None or np.any(np.isnan(q)):
            print("IK failed")
            return False
        
        # joint range check
        if np.any(q < self.jnt_min + tol) or np.any(q > self.jnt_max - tol):
            print("Joint range exceeded")
            return False
        
        # All checks passed
        return True
