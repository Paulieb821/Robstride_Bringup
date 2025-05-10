import serial
import time
from pynput import keyboard

PORT = '/dev/ttyACM0'
BAUD = 115200

gripper_open = False

def on_press(key):
    global gripper_open, ser
    
    if key == keyboard.Key.space:
        gripper_open = not gripper_open
        cmd = b'G' if gripper_open else b'H'
        try:
            ser.write(cmd)
            print(f"[SEND] {cmd!r} → {'OPEN' if gripper_open else 'CLOSE'}")
        except serial.SerialException as e:
            print("[ERROR] Failed to write:", e)
    
    elif key == keyboard.Key.esc:
        print("[EXIT] ESC pressed. Closing.")
        ser.close()
        return False
    
try:
    ser = serial.Serial(PORT, BAUD, timeout=0)
    time.sleep(2)  # Allow Arduino to reset
    print(f"[CONNECTED] {PORT} at {BAUD} baud. Press SPACE to toggle, ESC to quit.")
except Exception as e:
    print("[ERROR] Could not open port:", e)
    exit(1)

with keyboard.Listener(on_press=on_press) as listener:
    listener.join()