#!/usr/bin/env python3
"""
Smoke‐test your two‐servo gripper:

1) Prints whether serial link succeeded.
2) Lets you press keys to step open/close or toggle.
"""
from inputs import get_gamepad
import sys
import termios 
import tty
import threading
import time

from servocontrol import GripperController

gamepad_state = {
    'x':      0,    # raw ABS_X
    'y':      0,    # raw ABS_Y
    'hat_y':  0,    # raw ABS_HAT0Y (-1/0/1)
    'open':   0,    # BTN_SOUTH (A)
    'close':  0,    # BTN_EAST  (B)
    'toggle': 0,    # BTN_NORTH (X)
}

def _gamepad_listener():
    DEADZONE = 0.2
    while True:
        events = get_gamepad()
        for e in events:
            print(f"[GAMEPAD] ev_type={e.ev_type} code={e.code} state={e.state}")
            # Analog
            if e.ev_type == 'Absolute':                
                if e.code == 'ABS_X':
                    gamepad_state['x'] = e.state
                elif e.code == 'ABS_Y':
                    gamepad_state['y'] = e.state
                elif e.code == 'ABS_HAT0Y':
                    gamepad_state['hat_y'] = e.state
            # Buttons
            if e.ev_type == 'Key':
                if e.code == 'BTN_SOUTH':  # A
                    gamepad_state['open'] = e.state
                elif e.code == 'BTN_EAST':  # B
                    gamepad_state['close'] = e.state
                elif e.code == 'BTN_NORTH': # X
                    gamepad_state['toggle'] = e.state
        # rate limit
        time.sleep(0.005)

# start in background
threading.Thread(target=_gamepad_listener, daemon=True).start()

def main():
    try:
        gripper = GripperController(
            port="/dev/ttyUSB0",
            baud=9600,
            servo_ids=(8, 11),
            pulse_open   =[500, 2500],
            pulse_closed =[2500, 500],
            step_us=50
        )
        print("✔ Serial connection established.")
    except Exception as e:
        print(f"✘ Failed to connect: {e}")
        sys.exit(1)

    print("\nControls:")
    print("  o — open one step")
    print("  c — close one step")
    print("  t — full toggle")
    print("  q — quit\n")

    while True:
        
        if gamepad_state['open'] == 1:
            gamepad_state['open'] = 0
            gripper.open_step()

        if gamepad_state['close'] == 1:
            gamepad_state['close'] = 0
            gripper.close_step()

        if gamepad_state['toggle'] == 1:
            gamepad_state['toggle'] = 0
            gripper.toggle()

if __name__ == '__main__':
    main()
