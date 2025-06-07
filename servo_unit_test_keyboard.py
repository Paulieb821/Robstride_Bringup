#!/usr/bin/env python3
"""
Smoke‐test your two‐servo gripper:

1) Prints whether serial link succeeded.
2) Lets you press keys to step open/close or toggle.
"""

import sys
import termios 
import tty

from servocontrol import GripperController

def getch():
    """Read a single character (no Enter) from stdin."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def main():
    try:
        gr = GripperController(
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
        ch = getch().lower()
        if ch == 'o':
            gr.open_step()
            print("→ stepped open")
        elif ch == 'c':
            gr.close_step()
            print("→ stepped closed")
        elif ch == 't':
            gr.toggle()
            print("↔ toggled")
        elif ch == 'q':
            print("Exiting.")
            break
        else:
            print(f"(unmapped key: {repr(ch)})")

if __name__ == '__main__':
    main()
