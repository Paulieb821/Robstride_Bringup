import can
import robstride
import time
import numpy as np

with can.Bus() as bus:
    rs_client = robstride.Client(bus)

    # Disable to prevent any holding
    rs_client.disable(2)
    # rs_client.disable(3)
    # rs_client.disable(4)

    # First set the run mode to position
    rs_client.write_param(2, 'run_mode', robstride.RunMode.Current)
    rs_client.write_param(3, 'run_mode', robstride.RunMode.Operation)
    rs_client.write_param(4, 'run_mode', robstride.RunMode.Operation)
    
    # Then enable the motor
    rs_client.enable(2)
    # rs_client.enable(3)
    # rs_client.enable(4)

    # Next tell the motor to go to its zero position
    # Many calls including enable and write return the current state of the motor
    final_val = 3
    moveTime = 0.5
    startTime = time.time()
    startPos = rs_client.read_param(2, 'mechpos')
    while True:
        elapsed = time.time() - startTime
        pos_ref = startPos + (final_val - startPos)/moveTime * elapsed
        if elapsed > moveTime:
            pos_ref= final_val
        pos = rs_client.read_param(2, 'mechpos')
        vel = rs_client.read_param(2, 'mechvel')
        trq = -60*(pos-pos_ref) -3*vel
        cur = trq/(1.22*np.sqrt(2))
        rs_client.write_param(2, 'iq_ref', cur)
        print(pos)

    # while True:
    #     pos2 = rs_client.read_param(2, 'mechpos')
    #     #pos3 = rs_client.read_param(3, 'mechpos')
    #     #pos4 = rs_client.read_param(4, 'mechpos')
    #     #print("J2:", round(pos2, 2), "J3:", round(pos3, 2), "J4:", round(pos4, 2))
    #     print(pos2)

    # Finally deactivate the motor
    rs_client.disable(2)