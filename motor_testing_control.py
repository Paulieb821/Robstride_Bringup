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

# User Parameter
setpoint = 2
move_time = 1
deadzone_torque = 0.1
torque_limit = 0.6

# Gains
kd = 1
kp = 3

def trajectory(xf, T, time):
    if time < T:
        c2 = 3*xf/pow(T,2)
        c3 = -2*xf/pow(T,3)
        return (c2*pow(time,2) + c3*pow(time,3), 2*c2*time + 3*c3*pow(time,2))
    else:
        return (xf, 0)

def control_thread():
    start_time = time.time()
    while True:
        # Get elapsed time
        elapsed = time.time() - start_time
        pos_ref, vel_ref = trajectory(setpoint, move_time, elapsed)
        # Get motor state
        state = supervisor.get_actuators_state([1])
        if state:
            # Process position and velocity
            pos = math.radians(state[0].position)
            vel = math.radians(state[0].velocity)
            # Calculate torque using PD controller
            cmd_trq = kp*(pos_ref-pos) + kd*(vel_ref-vel)
            # Torque Limits
            if abs(cmd_trq) < deadzone_torque:
                cmd_trq = 0
            elif abs(cmd_trq) > torque_limit:
                cmd_trq = (cmd_trq/abs(cmd_trq))*torque_limit
            # Position Limits
            if abs(pos) > 3.1:
                cmd_trq = 0
            if abs(vel) > 1:
                cmd_trq = 0
            # Logging
            #cmd_trq = 0
            print(round(pos,2), round(vel,2), round(cmd_trq,2))
            print(round(pos_ref,2), round(vel_ref,2))
            # Send out command
            supervisor.command_actuators(
                [RobstrideActuatorCommand(actuator_id=1, position=0, velocity=0.0, torque=cmd_trq)]
            )
        time.sleep(0.01)

control = threading.Thread(target=control_thread)

control.start()
