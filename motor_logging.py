import can
import robstride
import time
import numpy as np
import subprocess

def setup_can_interface():
    try:
        # Check if can0 is already up
        result = subprocess.run(["ip", "link", "show", "can0"], capture_output=True, text=True)
        if "state UP" not in result.stdout:
            print("Setting up CAN interface...")
            subprocess.run(["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate", "1000000", "loopback", "off"], check=True)
            subprocess.run(["sudo", "ifconfig", "can0", "up"], check=True)
        else:
            print("CAN interface already up.")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to set up CAN interface: {e}")
    except FileNotFoundError:
        print("[ERROR] Required system commands not found. Make sure you are on a Linux system with CAN utilities installed.")

# Call setup function
setup_can_interface()

with can.Bus(interface='socketcan', channel='can0', bitrate=1000000) as bus:
    rs_client = robstride.Client(bus)
    time.sleep(0.5)

    motors_to_log = [1, 2, 3, 4]
    
    # Disable to prevent any holding
    for id in motors_to_log:
        motor_model = 1 if id != 2 else 2
        rs_client.disable(id, motor_model=motor_model)

    # Set the run mode to operaiton
    for id in motors_to_log:
        motor_model = 1 if id != 2 else 2
        rs_client.write_param(id, 'run_mode', robstride.RunMode.Operation, motor_model=motor_model)
    
    # Then enable the motor
    for id in motors_to_log:
        motor_model = 1 if id != 2 else 2
        rs_client.enable(id, motor_model=motor_model)

    while True:
        positions = []
        for id in motors_to_log:
            motor_model = 1 if id != 2 else 2   
            pos = rs_client.read_param(id, 'mechpos', motor_model=motor_model)
            positions.append(round(pos, 2))

        # Create a dynamic print string like "J1: 12.3 J2: 45.6 ..."
        output = ' '.join([f"J{motor_id}: {pos}" for motor_id, pos in zip(motors_to_log, positions)])
        print(output)