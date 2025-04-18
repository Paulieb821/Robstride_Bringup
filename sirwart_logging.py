import can
import robstride
import time
import numpy as np

with can.Bus() as bus:
    rs_client = robstride.Client(bus)

    # Disable to prevent any holding
    #rs_client.disable(1)
    rs_client.disable(2)
    rs_client.disable(3)
    rs_client.disable(4)

    # First set the run mode to position
    #rs_client.write_param(1, 'run_mode', robstride.RunMode.Operation)
    rs_client.write_param(2, 'run_mode', robstride.RunMode.Operation)
    rs_client.write_param(3, 'run_mode', robstride.RunMode.Operation)
    rs_client.write_param(4, 'run_mode', robstride.RunMode.Operation)
    
    # Then enable the motor
    #rs_client.enable(1)
    rs_client.enable(2)
    rs_client.enable(3)
    rs_client.enable(4)

    while True:
        #pos1 = rs_client.read_param(1, 'mechpos')
        pos2 = rs_client.read_param(2, 'mechpos')
        pos3 = rs_client.read_param(3, 'mechpos')
        pos4 = rs_client.read_param(4, 'mechpos')
        print("J2:", round(pos2, 2), "J3:", round(pos3, 2), "J4:", round(pos4, 2))
        #"J1:", round(pos1, 2), 