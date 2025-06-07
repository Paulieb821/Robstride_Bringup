#!/usr/bin/env python3
"""
gamepad_input_test.py — isolate and normalize left joystick input
"""

import threading
import time
from inputs import get_gamepad

# Shared state updated by listener thread
gamepad_state = {
    'x':     0,    # raw ABS_X
    'y':     0,    # raw ABS_Y
    'hat_y': 0,    # raw ABS_HAT0Y (-1/0/1)
}

def _gamepad_listener():
    DEADZONE = 0.1
    while True:
        events = get_gamepad()
        for e in events:
            # Analog sticks only
            if e.ev_type == 'Absolute':
                if e.code == 'ABS_X':
                    gamepad_state['x'] = e.state
                elif e.code == 'ABS_Y':
                    gamepad_state['y'] = e.state
                elif e.code == 'ABS_HAT0Y':
                    gamepad_state['hat_y'] = e.state
        time.sleep(0.005)

def normalize(value, center, halfrange):
    """Map raw [0..255] to [-1..1], clamped, with a deadzone."""
    norm = (value - center) / halfrange
    # clamp
    return max(-1.0, min(1.0, norm))

def main():
    # start listener thread
    threading.Thread(target=_gamepad_listener, daemon=True).start()

    # joystick calibration constants
    X_CENTER, X_HALFRANGE = 128, 128
    Y_CENTER, Y_HALFRANGE = 128, 128

    try:
        print("Testing gamepad input. Press Ctrl+C to quit.")
        while True:
            raw_x = gamepad_state['x']
            raw_y = gamepad_state['y']
            hat    = gamepad_state['hat_y']

            x_norm = normalize(raw_x, X_CENTER, X_HALFRANGE)
            y_norm = normalize(raw_y, Y_CENTER, Y_HALFRANGE)
            
            # apply deadzone
            if abs(x_norm) < 0.1:
                x_norm = 0.0
            if abs(y_norm) < 0.1:
                y_norm = 0.0

            print(f"RAW:  X={raw_x:3d}, Y={raw_y:3d}, HAT={hat:+d}   "
                  f"NORM: X={x_norm:+.2f}, Y={y_norm:+.2f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting.")

if __name__ == "__main__":
    main()





# #!/usr/bin/env python3
# """
# Script to determine the raw value ranges for joystick axes
# using the 'inputs' library.
# """

# import time
# from inputs import get_gamepad

# # Initialize min/max values
# min_x = float('inf')
# max_x = float('-inf')
# min_y = float('inf')
# max_y = float('-inf')

# print("Monitoring joystick inputs...")
# print("Move the joysticks through their full range of motion.")
# print("Press Ctrl+C to stop.")

# try:
#     while True:
#         events = get_gamepad()
#         for e in events:
#             # Only process Absolute events (analog sticks)
#             if e.ev_type == 'Absolute':
#                 if e.code == 'ABS_X':
#                     current_x = e.state
#                     min_x = min(min_x, current_x)
#                     max_x = max(max_x, current_x)
#                     print(f"ABS_X: {current_x} | Observed Range: [{min_x}, {max_x}]")
#                 elif e.code == 'ABS_Y':
#                     current_y = e.state
#                     min_y = min(min_y, current_y)
#                     max_y = max(max_y, current_y)
#                     # Note: ABS_Y is often inverted, min might be the highest value and max the lowest.
#                     # The script will correctly report the numerical min/max regardless.
#                     print(f"ABS_Y: {current_y} | Observed Range: [{min_y}, {max_y}]")
#                 # Optionally add other axes if needed, e.g., ABS_Z, ABS_RZ, etc.
#                 # elif e.code == 'ABS_Z':
#                 #     print(f"ABS_Z: {e.state}")
#                 # elif e.code == 'ABS_RZ':
#                 #     print(f"ABS_RZ: {e.state}")

#         # Small sleep to prevent high CPU usage
#         time.sleep(0.005)

# except KeyboardInterrupt:
#     print("\nStopping monitoring.")
#     print("-" * 30)
#     print("Final Observed Ranges:")
#     print(f"ABS_X: [{min_x}, {max_x}]")
#     print(f"ABS_Y: [{min_y}, {max_y}]")
#     print("-" * 30)