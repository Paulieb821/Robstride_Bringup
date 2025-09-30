import can
import robstride
import time
import numpy as np

with can.Bus(interface='socketcan', channel='can0', bitrate=1000000) as bus:
    rs_client = robstride.Client(bus)

    motors_id = [1]

    # Disable to prevent any holding
    for id in motors_id:
        rs_client.disable(id)
        motor_model = 1 if id != 2 else 2
        rs_client.write_param(id, 'run_mode', robstride.RunMode.Operation, motor_model=motor_model)
        print(rs_client.read_param(id, 'spd_filt_gain'))
        # rs_client.write_param(id, 'spd_filt_gain', 0.5)