import can
import robstride
import time
import numpy as np

with can.Bus() as bus:
    rs_client = robstride.Client(bus)

    motors_to_zero = [1, 2, 3, 4]

    # Disable to prevent any holding
    for id in motors_to_zero:
        rs_client.disable(id)
        rs_client.write_param(id, 'run_mode', robstride.RunMode.Operation)
        rs_client.zero_pos(id)
    