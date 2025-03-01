import numpy as np
from PID_gain_calculator import pid_gains

class PID_Controller:

    # Initalize Controller
    def __init__(self, timestep, inertia, bandwidth, Kd_correction, anti_windup_trq, pos_deadzone, vel_deadzone, initial_pos, initial_vel):
        # Populate parameters
        self.pos_deadzone = pos_deadzone
        self.vel_deadzone = vel_deadzone
        self.J = inertia
        self.anti_windup = anti_windup_trq
        self.T = timestep
        # Calculate gains for regulator and estimator
        K, L, self.Ad, self.Bd, self.Cd = pid_gains(bandwidth, timestep, 0.0, 1/inertia, 0.0, 0.0)
        self.K = K[0]
        self.K[1] = self.K[1] * Kd_correction
        self.L = L[0]
        # Initialize estimator
        self.xhat = np.array([[initial_pos], [initial_vel]])
        # Initialize integrator
        self.sigma = 0
    
    # Run Update Loop
    def update_controller(self, pos, pos_ref, vel_ref, acc_ref):
        # Generate torque command with deadzoning
        if abs(self.xhat[0,0]-pos_ref) < self.pos_deadzone and abs(vel_ref) <= self.vel_deadzone:
            cmd_trq = 0
        else:
            cmd_trq = self.K[0]*(pos_ref-self.xhat[0,0]) + self.K[1]*(vel_ref-self.xhat[1,0]) - self.K[2]*self.sigma + self.J*acc_ref
        # Update state estimator
        xhat = self.Ad @ xhat + self.Bd * cmd_trq - self.L * (self.Cd @ xhat - pos)
        # Update integrator
        if abs(cmd_trq) < 6:
            sigma = sigma + self.T*(pos - pos_ref)
        # Return command torque
        return cmd_trq

