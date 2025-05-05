
from gpiozero import AngularServo
from time import sleep

servo = AngularServo(18, min_angle=0, max_angle=270, min_pulse_width=0.5/1000,max_pulse_width=2.5/ 1000)
while True:
    servo.angle = 0  # Set the servo to 0 degrees
    sleep(1)  # Wait for 1 second
    servo.angle = 90  # Set the servo to 90 degrees
    sleep(1)  # Wait for 1 second
    servo.angle = 180  # Set the servo to 180 degrees
    sleep(1)  # Wait for 1 second
    servo.angle = 270  # Set the servo to 270 degrees
    sleep(1)  # Wait for 1 second    


    # for angle in range(0, 271, 1):  # sweep from 0° to 270°
    #     servo.angle = angle
    #     sleep(0.001)

    # for angle in range(270, -1, -1):  # sweep back from 270° to 0°
    #     servo.angle = angle
    #     sleep(0.001)


