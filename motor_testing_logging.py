import argparse
import math
import time
import os
import threading
import numpy as np
from actuator import RobstrideActuator, RobstrideActuatorConfig, RobstrideActuatorCommand

# Cleanup
#os.system('clear')

# Create Supervisor
supervisor = RobstrideActuator(ports=['/dev/ttyUSB0'], py_actuators_config=[
    (1, RobstrideActuatorConfig(1)),    # J1
    (2, RobstrideActuatorConfig(3)),    # J2
    (3, RobstrideActuatorConfig(1)),    # J3
    ])

supervisor.run_main_loop(1)
time.sleep(1)
supervisor.enable(1)
supervisor.enable(2)
supervisor.enable(3)

def monitor_thread():

    loop_start = time.time()
    while True:
        state = supervisor.get_actuators_state([2])
        if state:
            pos = math.radians(state[0].position)
            print("Motor 1 Position:", round(pos, 2))
            # print("Motor 4 Position:", round(pos, 2))
            # print("Joint Position:", round(pos/3, 2))
            # print("")
        else:
            print("failed")
        time.sleep(0.01)

        # state = supervisor.get_actuators_state([1,2])
        # if state and len(state)==2:
        #     print("J1:", round(np.deg2rad(state[0].position), 2), "J2:", round(np.deg2rad(state[1].position), 2))
        # else:
        #     print(len(state))
        # time.sleep(0.01)


        # state = supervisor.get_actuators_state([1,2,3])
        # if state and len(state)==3:
        #     print("J1:", round(state[0].position), "J2:", round(state[1].position), "J3:", round(state[2].position))
        # else:
        #     if state:
        #         if len(state) == 1:
        #             print("Failed. Working Motors: ", state[0].actuator_id)
        #         if len(state) == 2:
        #             print("Failed. Working Motors: ", state[0].actuator_id, state[1].actuator_id)
        
        # time.sleep(0.01)

monitor = threading.Thread(target=monitor_thread)

monitor.start()

# J1 --> 3
# J2 --> 1
# J3 --> 2