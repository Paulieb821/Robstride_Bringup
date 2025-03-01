import numpy as np
from numpy.linalg import inv
from scipy.linalg import expm
from scipy.signal import place_poles

def pd_gains(lambda_val, T, b1, b0, a1, a0):
    # DT Poles
    gammaR = np.exp(-lambda_val * T)
    gammaE = np.exp(-lambda_val * 5 * T)

    # Observable Form State Space Model
    A = np.array([[-a1, 1], [-a0, 0]])
    B = np.array([[b1], [b0]])
    C = np.array([[1, 0]])

    # Convert to State Variables that Represent Position and Velocity
    Trans = np.array([[1,0], [-a1,1]])
    A = Trans @ A @ inv(Trans)
    B = Trans @ B
    C = C @ inv(Trans)

    # Discrete-Time State Space Model (using matrix exponential)
    Ad = expm(A * T)

    # Numerical Integration for Bd
    Bd = np.zeros_like(B)
    for i in np.arange(0, T, T / 50):  # T/20 steps
        Bd += expm(A * i) @ B * (T / 50)

    Cd = C

    # Pole placement for Discrete-Time System (Regulator and Observer gains)
    # Regulator Gain (Kd)
    place_reg = place_poles(Ad, Bd, np.array([gammaR, gammaR+0.001]))
    K = place_reg.gain_matrix

    # Observer Gain (Ld)
    place_obs = place_poles(Ad.T, Cd.T, np.array([gammaE, gammaE+0.001]))
    L = place_obs.gain_matrix.T  # Transpose to get observer gain

    return K, L, Ad, Bd, Cd



