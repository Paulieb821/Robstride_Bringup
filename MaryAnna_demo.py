"""Example of moving a motor using the supervisor."""

import argparse
import time
import os

from actuator import RobstrideActuator, RobstrideActuatorConfig, RobstrideActuatorCommand
from dataclasses import dataclass
import threading
import logging

logging.getLogger().setLevel(logging.INFO)


@dataclass
class ActuatorState:
    actuator_id: int
    online: bool
    position: float
    velocity: float
    torque: float
    temperature: float


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port-name", type=str, default="/dev/ttyCH341USB0")
    parser.add_argument("--motor-id", type=int, default=1)
    parser.add_argument("--motor-type", type=int, default=4)
    parser.add_argument("--sleep", type=float, default=0.1)
    parser.add_argument("--period", type=float, default=10.0)
    parser.add_argument("--amplitude", type=float, default=1.0)
    parser.add_argument("--update_loop_interval_ms", type=int, default=10)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    _amplitude = args.amplitude
    _period = args.period
    supervisor = RobstrideActuator(
        ports=[args.port_name],
        py_actuators_config=[(args.motor_id, RobstrideActuatorConfig(args.motor_type))],
    )

    supervisor.disable(args.motor_id)
    target_fps = 30
    frame_time = 1.0 / target_fps
    start_time = time.time()
    current_state = None
    supervisor.run_main_loop(interval_ms=args.update_loop_interval_ms)

    def monitor_thread():
        nonlocal start_time, current_state
        while True:
            loop_start = time.time()

            state = supervisor.get_actuators_state([args.motor_id])
            if state:
                pos = state[0].position
                current_state = state[0].position

            elapsed = time.time() - start_time
            if elapsed >= 1.0:
                fps = 1.0 / ((time.time() - start_time) / target_fps)
                start_time = time.time()

            sleep_time = frame_time - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    pos = 0
    torque = 0

    def control_thread():
        nonlocal pos, current_state, torque
        while True:
            command = input("Enter command (enable/disable/quit | z,x,d,a): ").lower()
            if command in ["z", "x", "d", "a"]:
                if command == "z":
                    torque += 1
                elif command == "x":
                    torque -= 1
                elif command == "d":
                    pos += 5
                elif command == "a":
                    pos -= 5
                print(pos, torque)
                supervisor.command_actuators(
                    [
                        RobstrideActuatorCommand(
                            actuator_id=args.motor_id,
                            position=pos,
                            velocity=10,
                            torque=torque,
                        )
                    ]
                )
            elif command == "enable":
                supervisor.enable(args.motor_id)
                print("Motor enabled")
            elif command == "disable":
                supervisor.disable(args.motor_id)
                print("Motor disabled")
            elif command == "quit":
                supervisor.command_actuators(
                    [
                        RobstrideActuatorCommand(
                            actuator_id=args.motor_id,
                            position=pos,
                            velocity=0,
                            torque=0,
                        )
                    ]
                )
                time.sleep(2)  # Allow decelerate
                supervisor.disable(args.motor_id)
                time.sleep(0.5)  # Allow exit
                print("Exiting...")
                os._exit(0)
            else:
                print("Invalid command")

    monitor = threading.Thread(target=monitor_thread)
    control = threading.Thread(target=control_thread)

    try:
        monitor.start()
        control.start()
        monitor.join()
        control.join()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")


if __name__ == "__main__":
    # python -m examples.supervisor
    main()