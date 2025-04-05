def cubic_trajectory(start_pos, end_pos, final_time, current_time):
    # Shorter names for easier writing
    xo = start_pos
    xf = end_pos
    tf = final_time
    time = current_time
    # Compute trajectory
    if time < tf:
        c0 = xo
        c2 = 3*(xf - xo)/pow(tf,2)
        c3 = -2*(xf - xo)/pow(tf,3)
        return (c0 + c2*pow(time,2) + c3*pow(time,3), 2*c2*time + 3*c3*pow(time,2), 2*c2 + 6*c3*time)
    else:
        return (xf, 0, 0)
    
def quintic_trajectory(start_pos, end_pos, final_time, current_time):
    # Shorter names for easier writing
    xo = start_pos
    xf = end_pos
    tf = final_time
    time = current_time
    # Calculate the coefficients for the quintic polynomial
    a0 = xo
    a1 = 0  # initial velocity is 0
    a2 = 0  # initial acceleration is 0
    a3 = (10 * (xf - xo)) / (tf ** 3)
    a4 = (-15 * (xf - xo)) / (tf ** 4)
    a5 = (6 * (xf - xo)) / (tf ** 5) 

    if time < tf:
        # Compute position and velocity at a given time
        position = a0 + a1 * time + a2 * time**2 + a3 * time**3 + a4 * time**4 + a5 * time**5
        velocity = a1 + 2 * a2 * time + 3 * a3 * time**2 + 4 * a4 * time**3 + 5 * a5 * time**4
        accel = 2*a2 + 6*a3*time + 12*a4*time**2 + 20*a5*time**3
        return position, velocity, accel
    else:
        # At the final time, return the final position and velocity = 0
        return xf, 0,0 
    
    