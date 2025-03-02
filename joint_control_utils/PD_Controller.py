import numpy as np
from joint_control_utils.PD_gain_calculator import pd_gains

class PD_Controller:

    # Initalize Controller
    def __init__(self, timestep, gear_ratio, inertia, bandwidth, Kd_correction, anti_windup_trq, pos_deadzone, vel_deadzone, initial_pos, initial_vel, joint_limits):
        # Populate parameters
        self.pos_deadzone = pos_deadzone
        self.vel_deadzone = vel_deadzone
        self.J = inertia
        self.anti_windup = anti_windup_trq
        self.T = timestep
        self.gear_ratio = gear_ratio
        self.joint_limits = joint_limits
        # Calculate gains for regulator and estimator
        K, L, self.Ad, self.Bd, self.Cd = pd_gains(bandwidth, timestep, 0.0, 1.0, 0.0, 0.0)
        self.K = K[0]
        self.K[1] = self.K[1] * Kd_correction
        self.L = L[0]
        # Initialize estimator
        self.xhat = np.array([[initial_pos/gear_ratio], [initial_vel/gear_ratio]])
    
    # Run Update Loop
    def update_controller(self, pos, vel, pos_ref, vel_ref, acc_ref, comp_trq):
        # Change encoder readings to joint space
        pos = pos/self.gear_ratio
        vel = vel/self.gear_ratio
        # Generate torque command with deadzoning
        if abs(self.xhat[0,0]-pos_ref) < self.pos_deadzone and abs(vel_ref) < self.vel_deadzone:
            cmd_trq = 0
        elif pos > self.joint_limits[0] or pos < self.joint_limits[1]:
            cmd_trq = 0
        else:
            cmd_trq = self.K[0]*(pos_ref-self.xhat[0,0]) + self.K[1]*(vel_ref-self.xhat[1,0]) + acc_ref
            print("Tp: ", round(self.K[0]*(pos_ref-self.xhat[0,0]),2))
            print("Td: ", round(self.K[1]*(vel_ref-self.xhat[1,0]),2))
            print("Tg: ", round(comp_trq*np.sin(pos), 2))
        # Update state estimator
        self.xhat = self.Ad @ self.xhat + self.Bd * cmd_trq - self.L * (self.Cd @ self.xhat - pos)
        # Return command torque
        return (self.J*cmd_trq + comp_trq*np.sin(pos))/self.gear_ratio

