import can
import robstride
import time
import numpy as np
import subprocess
import sys
import matplotlib.pyplot as plt  # <-- Add this for plotting

def check_can_interface_exists(interface_name):
    try:
        result = subprocess.run(["ip", "a"], capture_output=True, text=True, check=True)
        return interface_name in result.stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        print(f"[ERROR] Could not check for network interfaces. Is 'ip' command available?")
        return False

def setup_can_interface():
    try:
        if not check_can_interface_exists("can0"):
            print("[ERROR] CAN interface 'can0' not found!")
            sys.exit(1)

        result = subprocess.run(["ip", "link", "show", "can0"], capture_output=True, text=True)
        if "state UP" not in result.stdout:
            print("Bringing up CAN interface 'can0'...")
            subprocess.run(
                ["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate", "1000000", "loopback", "off"],
                check=True
            )
            subprocess.run(["sudo", "ip", "link", "set", "can0", "up"], check=True)
            print("'can0' is now up.")
        else:
            print("CAN interface 'can0' is already up.")

    except subprocess.CalledProcessError as e:
        print(f"[ERROR] A command failed during CAN setup: {e}")
        sys.exit(1)
    except FileNotFoundError:
        print("[ERROR] A system command was not found. Ensure you are on Linux with 'iproute2' installed.")
        sys.exit(1)

# --- Main ---
print("--- Starting Motor Logging ---")
setup_can_interface()

# Data storage for plotting
log_times = []
log_velocities = {1: [], 2: [], 3: [], 4: []}

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

        print("\n--- Reading Motor Velocities ---")
        start_time = time.time()
        while True:
            current_time = time.time() - start_time
            log_times.append(current_time)
            velocities = []
            st_time = time.time()
            for id in motors_to_log:
                vel = rs_client.read_param(id, 'mechvel')
                log_velocities[id].append(vel)
                velocities.append(round(vel, 2))
            elapsed_time = time.time() - st_time
            output = ' '.join([f"J{motor_id}: {vel}" for motor_id, vel in zip(motors_to_log, velocities)])
            print(f"\r{output} | Loop time: {elapsed_time:.3f}", end="")
            time.sleep(0.1)

except can.CanError as e:
    print(f"\n[CRITICAL] A CAN error occurred: {e}")
except KeyboardInterrupt:
    print("\n--- Shutting down ---")
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

    # --- Plotting velocities ---
    print("Plotting joint velocities over time...")

    y_axis_limits = (-1, 1)  # <-- Adjust this as needed; set to None for auto

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    fig.suptitle("Joint Velocities Over Time", fontsize=16)

    for idx, joint_id in enumerate([1, 2, 3, 4]):
        row, col = divmod(idx, 2)
        ax = axes[row][col]
        if joint_id in log_velocities and len(log_velocities[joint_id]) > 0:
            ax.plot(log_times, log_velocities[joint_id], label=f'Joint {joint_id}', color=f'C{idx}')
            ax.set_title(f'Joint {joint_id}')
            ax.set_ylabel("Velocity")
            if y_axis_limits:
                ax.set_ylim(y_axis_limits)
        else:
            ax.set_title(f'Joint {joint_id} (no data)')
            ax.set_xticks([])
            ax.set_yticks([])

    for ax in axes[1]:
        ax.set_xlabel("Time (s)")

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


    sys.exit(0)
