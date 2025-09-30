import time
import numpy as np
import serial
import sys

from UI_utils.keyboard_listener import Keyboard_Listener
from servocontrol import GripperController 

########################
# CONFIGURATION SECTION
########################
 
# Gripper control
using_gripper = True

command_rate = 20

########################
# INITIALIZATION
#################w#######


# Control helpers
kb = Keyboard_Listener()

# Gripper setup
if using_gripper:
    try:
        gripper = GripperController(            
            port="/dev/ttyUSB0",
            baud=9600,
            servo_ids=(8, 11),     # DOUBLE CHECK WHAT IS ON HARDWARE
            pulse_open = 500,      # swap these if direction is reversed
            pulse_closed = 2000
        )
    except Exception as e:
        print(f"[WARNING] Gripper init failed: {e}")
        using_gripper = False

########################
# MAIN CONTROL LOOP
########################
while True:
    
    if using_gripper:        
        if kb.key_states['o']:
            kb.key_states['o'] = False
            gripper.open_step()
        elif kb.key_states['c']:
            kb.key_states['c'] = False
            gripper.close_step()
        elif kb.key_states['space']:
            kb.key_states['space'] = False
            gripper.toggle()
        
    time.sleep(1 / command_rate)
