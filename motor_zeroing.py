import can
import robstride
import time
import numpy as np

with can.Bus(interface='socketcan', channel='can0', bitrate=1000000) as bus:
    rs_client = robstride.Client(bus)

    motors_to_zero = [1, 2, 3, 4]

    # Disable to prevent any holding
    for id in motors_to_zero:
        rs_client.disable(id)
        motor_model = 1 if id != 2 else 2
        rs_client.write_param(id, 'run_mode', robstride.RunMode.Operation, motor_model=motor_model)
        rs_client.zero_pos(id)
    