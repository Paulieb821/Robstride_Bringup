from actuator import RobstrideActuator, RobstrideActuatorConfig, RobstrideConfigureRequest

def zero_actuator(actuator_id):
    # Create a configuration request with the zero_position flag set
    supervisor = RobstrideActuator(ports=['/dev/ttyUSB0'], py_actuators_config=[
    (1, RobstrideActuatorConfig(1)),    # J1
    (7, RobstrideActuatorConfig(3)),    # J2
    (3, RobstrideActuatorConfig(1)),    # J3
    ])
    config_req = RobstrideConfigureRequest(
        actuator_id=actuator_id,
        kp=None,             # Use default or specific values if needed
        kd=None,
        max_torque=None,
        torque_enabled=None,
        zero_position=True,
        new_actuator_id=None
    )
    # Send the configuration request to zero the actuator
    success = supervisor.configure_actuator(config_req)
    print(success)
    if success:
        print(f"Actuator {actuator_id} configure success.")
    else:
        print(f"Failed to configure motor {actuator_id}.")

if __name__ == "__main__":
    actuator_id = 1 
    zero_actuator(actuator_id)