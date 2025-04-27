import pigpio
import time

class ServoController:
    def __init__(self, pi, gpio_pin, min_angle=0, max_angle=270, min_pulse=500, max_pulse=2500):
        """
        pi: pigpio.pi() object
        gpio_pin: GPIO pin for the servo
        min_angle: minimum angle
        max_angle: maximum angle 
        min_pulse: pulse width at min_angle (μs)
        max_pulse: pulse width at max_angle (μs)
        these  values are based on https://www.hiwonder.com/products/hps-3527sg
        """
        self.pi = pi
        self.gpio_pin = gpio_pin
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse

        self.pi.set_mode(gpio_pin, pigpio.OUTPUT)
        self.stop()

    def set_angle(self, angle):
        if angle < self.min_angle:
            angle = self.min_angle
        if angle > self.max_angle:
            angle = self.max_angle

        # Map angle to pulse width
        pulse_width = self.min_pulse + (angle - self.min_angle) / (self.max_angle - self.min_angle) * (self.max_pulse - self.min_pulse)
        self.pi.set_servo_pulsewidth(self.gpio_pin, pulse_width)

    def stop(self):
        self.pi.set_servo_pulsewidth(self.gpio_pin, 0)

if __name__ == "__main__":
    pi = pigpio.pi()
    if not pi.connected:
        print("Failed to connect to pigpio daemon!")
        exit()

    # Servo 1 on GPIO 17, 0° to 270°
    servo1 = ServoController(pi, 17)
    # Servo 2 on GPIO 18, 0° to 270°
    servo1 = ServoController(pi, 17)

    try:
        while True:
            for angle in range(0, 271, 15):  # sweep from 0° to 270°
                servo1.set_angle(angle)
                time.sleep(0.05)

            for angle in range(270, -1, -15):  # sweep back from 270° to 0°
                servo1.set_angle(angle)
                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        servo1.stop()
        pi.stop()
