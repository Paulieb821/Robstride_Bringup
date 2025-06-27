import can
import robstride
import time
import numpy as np
import subprocess
import sys

def check_can_interface_exists(interface_name):
    """Checks if a given network interface exists."""
    try:
        # Use 'ip a' which is a more modern command than ifconfig
        result = subprocess.run(["ip", "a"], capture_output=True, text=True, check=True)
        return interface_name in result.stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        print(f"[ERROR] Could not check for network interfaces. Is 'ip' command available?")
        return False

def setup_can_interface():
    """Sets up the can0 interface if it exists but is not up."""
    try:
        # 1. First, check if the can0 interface even exists.
        if not check_can_interface_exists("can0"):
            print("[ERROR] CAN interface 'can0' not found!")
            print("        Please check kernel messages with 'dmesg | grep can' to see if the adapter was detected.")
            sys.exit(1) # Exit the script if the interface isn't there

        # 2. Check if can0 is already up
        result = subprocess.run(["ip", "link", "show", "can0"], capture_output=True, text=True)
        if "state UP" not in result.stdout:
            print("Bringing up CAN interface 'can0'...")
            # Set bitrate and other properties
            subprocess.run(
                ["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate", "1000000", "loopback", "off"],
                check=True
            )
            # Bring the interface up using the 'ip' command
            subprocess.run(["sudo", "ip", "link", "set", "can0", "up"], check=True)
            print("'can0' is now up.")
        else:
            print("CAN interface 'can0' is already up.")

    except subprocess.CalledProcessError as e:
        print(f"[ERROR] A command failed during CAN setup: {e}")
        print(f"        Command was: '{' '.join(e.cmd)}'")
        if e.stderr and "RTNETLINK answers: Connection timed out" in e.stderr:
             print("        This timeout often means the kernel driver has an issue. Try running 'sudo modprobe gs_usb'.")
        sys.exit(1) # Exit on error
    except FileNotFoundError:
        print("[ERROR] A system command was not found. Ensure you are on Linux with 'iproute2' installed.")
        sys.exit(1)

# --- Main part of your script ---
print("--- Starting Motor Logging ---")
setup_can_interface()

try:
    with can.Bus(interface='socketcan', channel='can0', bitrate=1000000) as bus:
        print("Successfully opened CAN bus.")
        rs_client = robstride.Client(bus)
        time.sleep(0.5)

        motors_to_log = [1, 2, 3, 4]
        
        print("Disabling motors to release any holding torque...")
        for id in motors_to_log:
            motor_model = 1 if id != 2 else 2
            rs_client.disable(id, motor_model=motor_model)

        print("Setting run mode to 'Operation'...")
        for id in motors_to_log:
            motor_model = 1 if id != 2 else 2
            rs_client.write_param(id, 'run_mode', robstride.RunMode.Operation, motor_model=motor_model)
        
        print("Enabling motors...")
        for id in motors_to_log:
            motor_model = 1 if id != 2 else 2
            rs_client.enable(id, motor_model=motor_model)

        print("\n--- Reading Motor Positions ---")
        while True:
            positions = []
            for id in motors_to_log:
                pos = rs_client.read_param(id, 'mechpos')
                positions.append(round(pos, 2))

            output = ' '.join([f"J{motor_id}: {pos}" for motor_id, pos in zip(motors_to_log, positions)])
            print(f"\r{output}", end="") # Use carriage return to print on the same line
            time.sleep(0.1)

except can.CanError as e:
    print(f"\n[CRITICAL] A CAN error occurred: {e}")
    print("           Please ensure the CAN adapter is connected and the interface is up.")
except KeyboardInterrupt:
    print("\n--- Shutting down ---")
    # It's good practice to try and disable motors on exit
    try:
        motors_to_log = [1, 2, 3, 4]
        with can.Bus(interface='socketcan', channel='can0') as bus:
            rs_client = robstride.Client(bus)
            for id in motors_to_log:
                motor_model = 1 if id != 2 else 2
                rs_client.disable(id, motor_model=motor_model)
            print("Motors disabled.")
    except Exception as e:
        print(f"Could not disable motors on exit: {e}")
    sys.exit(0)
