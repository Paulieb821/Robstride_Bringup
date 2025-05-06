import can
import robstride
import time
import numpy as np

with can.Bus() as bus:
    rs_client = robstride.Client(bus)
    time.sleep(0.5)

    motors_to_log = [1, 2, 3, 4]

    # Disable to prevent any holding
    for id in motors_to_log:
        rs_client.disable(id)

    # First set the run mode to position
    for id in motors_to_log:
        rs_client.write_param(id, 'run_mode', robstride.RunMode.Position)
    
    # Then enable the motor
    for id in motors_to_log:
        rs_client.enable(id)

    while True:
        positions = []
        for id in motors_to_log:
            pos = rs_client.read_param(id, 'mechpos')
            positions.append(round(pos, 2))

        # Create a dynamic print string like "J1: 12.3 J2: 45.6 ..."
        output = ' '.join([f"J{motor_id}: {pos}" for motor_id, pos in zip(motors_to_log, positions)])
        print(output)