import can
import robstride
import time
import numpy as np

with can.Bus() as bus:
    rs_client = robstride.Client(bus)

    # Parameters
    motorID = 2
    trq_amp = (0.224*0.3/2 + 1.1*0.3) * 9.81
    Kt = np.sqrt(2)*1.22   # Nm/Arms

    # Disable to prevent any holding
    rs_client.disable(motorID)

    # First set the run mode to position
    rs_client.write_param(motorID, 'run_mode', robstride.RunMode.Current)
    
    # Then enable the motor
    rs_client.enable(motorID)

    # Orient the arm downwards and record position
    bottomPos = rs_client.read_param(motorID, 'mechpos')

    while True:
        relpos = rs_client.read_param(motorID, 'mechpos') - bottomPos
        cmd_trq = trq_amp * np.sin(relpos)
        print("Position:", round(relpos,2), "Command Torque:", round(cmd_trq,2))
        cmd_cur = cmd_trq/Kt
        rs_client.write_param(motorID, 'iq_ref', cmd_cur)


    # Finally deactivate the motor
    rs_client.disable(2)