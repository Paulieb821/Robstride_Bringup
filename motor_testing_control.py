import argparse
import math
import time
import os
import threading
 
from actuator import RobstrideActuator, RobstrideActuatorConfig, RobstrideActuatorCommand, RobstrideConfigureRequest

# Cleanup
#os.system('clear')

# Create Supervisor
supervisor = RobstrideActuator(ports=['/dev/ttyUSB0'], py_actuators_config=[
    (1, RobstrideActuatorConfig(1)),    # J1
    (7, RobstrideActuatorConfig(3)),    # J2
    (3, RobstrideActuatorConfig(1)),    # J3
    ])

supervisor.run_main_loop(1)

print("Startup sequence")
time.sleep(1)
supervisor.enable(1)
supervisor.enable(7)
supervisor.enable(3)
print("Motor enabled")


def control_thread():
    while True:
        supervisor.command_actuators(
            [RobstrideActuatorCommand(actuator_id=1, position=0, velocity=0.1, torque=0.05)]
        )
        time.sleep(0.1)

control = threading.Thread(target=control_thread)

control.start()