from actuator import RobstrideActuator, RobstrideActuatorConfig, RobstrideActuatorCommand, RobstrideConfigureRequest

# Create Supervisor
supervisor = RobstrideActuator(ports=['/dev/ttyUSB0'], py_actuators_config=[
    (1, RobstrideActuatorConfig(1)),    # J1
    (7, RobstrideActuatorConfig(3)),    # J2
    (3, RobstrideActuatorConfig(1)),    # J3
    ])

# Send Request
config_req = RobstrideConfigureRequest(
        actuator_id=1,
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