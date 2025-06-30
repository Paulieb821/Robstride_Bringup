import can
import robstride
import time
import numpy as np
import pandas as pd

with can.Bus() as bus:
    rs_client = robstride.Client(bus)

    # Parameters
    motorID = 1
    motorKT = 1.22  # Torque constant of motor Nm/Arms (1.22 for RS01 and RS02, )
    testTime = 10   # Seconds
    timestep = 0.05 # Seconds
    amp = 0.5       # Torque amplitude in Nm
    freq = 1.5      # Current sine wave in Hz

    # Enable and Zero Motor
    rs_client.disable(motorID)
    rs_client.zero_pos(motorID)
    rs_client.write_param(motorID, 'run_mode', robstride.RunMode.Current)
    rs_client.enable(motorID)
    time.sleep(1)

    # Set Up Collection Arrays
    timeArr = []
    torqueArr = []
    posArr = []

    # Run sinusoidal data collection
    print("Test Starting")
    loopCount = 0
    elapsed = 0
    startTime = time.perf_counter()

    while True:
        # Command new current
        torque_cmd = amp*np.sin(2*np.pi*freq*elapsed)
        cur_cmd = torque_cmd/(motorKT*np.sqrt(2))
        rs_client.write_param(motorID, 'iq_ref', cur_cmd)

        # Read back params
        timeArr.append(time.perf_counter() - startTime)
        torqueArr.append(rs_client.read_param(motorID, 'iqf') * motorKT * np.sqrt(2))
        posArr.append(rs_client.read_param(motorID, 'mechpos'))

        # Sleep for the correct amount of time for even timing
        loopCount += 1
        elapsed = time.perf_counter() - startTime
        sleep_time = timestep * loopCount - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            print("Loop took too long to process - ending test, consider increasing timestep")
            break

    print("Test finished")

    # Finally deactivate the motor
    rs_client.disable(motorID)

    # Export data to csv
    df = pd.DataFrame({
        'time': timeArr,
        'torque': torqueArr,
        'position': posArr
    })
    df.to_csv('sys_id_data.csv', index=False)