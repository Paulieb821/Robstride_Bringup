from joint_control_utils.PID_gain_calculator import pid_gains

inertia = 0.1
K, L, Ad, Bd, Cd = pid_gains(10, 0.01, 0.0, 1/inertia, 0.0, 0.0)
K = K[0]
print(K)

K, L, Ad, Bd, Cd = pid_gains(10, 0.01, 0.0, 1, 0.0, 0.0)
K = K[0] * inertia
print(K)
