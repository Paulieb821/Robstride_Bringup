import can
import robstride
import time
import numpy as np

with can.Bus(interface='socketcan', channel='can0', bitrate=1000000) as bus:
    rs_client = robstride.Client(bus)
    time.sleep(0.5)

    motors_to_log = [1, 2, 3, 4]

    # Disable to prevent any holding
    for id in motors_to_log:
        rs_client.disable(id)

    # Set the run mode to operaiton
    for id in motors_to_log:
        rs_client.write_param(id, 'run_mode', robstride.RunMode.Operation)
    
    # Then enable the motor
    for id in motors_to_log:
        rs_client.enable(id)

    positions = []
    for id in motors_to_log:
        if id == 1 or id == 3 or id == 4:
            rs_client.write_param(id, 'loc_kp', 60.0)
        pos = rs_client.read_param(id, 'loc_kp')
        positions.append(round(pos, 2))

    # Create a dynamic print string like "J1: 12.3 J2: 45.6 ..."
    output = ' '.join([f"J{motor_id}: {pos}" for motor_id, pos in zip(motors_to_log, positions)])
    print(output)