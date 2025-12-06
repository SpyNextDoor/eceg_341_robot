import machine
import math
import time

class Motor:
    def __init__(self, m1a_pin=8, m1b_pin=9, m2a_pin=10, m2b_pin=11, freq=8000, bias=0.03, wheel_base=13):
        # Set up PWM pins
        self.M1A = machine.PWM(machine.Pin(m1a_pin))
        self.M1B = machine.PWM(machine.Pin(m1b_pin))
        self.M2A = machine.PWM(machine.Pin(m2a_pin))
        self.M2B = machine.PWM(machine.Pin(m2b_pin))

        # Set PWM frequency
        self.M1A.freq(freq)
        self.M1B.freq(freq)
        self.M2A.freq(freq)
        self.M2B.freq(freq)

        # Parameters
        self.bias = bias
        self.wheel_base = wheel_base

        # Split bias for left/right motors
        self.left_bias = 1.0 - bias / 2
        self.right_bias = 1.0 + bias / 2

    def drive(self, v_robot, omega):
        """Drive the robot given linear velocity (v_robot) and angular velocity (omega)."""
        # Compute individual wheel velocities

        v_left = v_robot + (omega * self.wheel_base / 2)
        v_right = v_robot - (omega * self.wheel_base / 2)

        # print(f"v_left: {v_left:.3f}, v_right: {v_right:.3f}")

        # Convert to PWM duty cycles
        duty_left = abs(int(1168 * v_left)) + 4509
        duty_right = abs(int(1168 * v_right)) + 4509

        # Clamp to max PWM range
        duty_left = min(duty_left, 65535)
        duty_right = min(duty_right, 65535)

        # print(f"duty_left: {duty_left:.3f}, duty_right: {duty_right:.3f}")

        # Set LEFT motor direction
        if v_left >= 0:
            self.M1A.duty_u16(int(duty_left * self.left_bias))
            self.M1B.duty_u16(0)
            
        else:
            self.M1A.duty_u16(0)
            self.M1B.duty_u16(int(duty_left * self.left_bias))

        # Set RIGHT motor direction
        if v_right >= 0:
            self.M2A.duty_u16(int(duty_right * self.right_bias))
            self.M2B.duty_u16(0)
            
        else:
            self.M2A.duty_u16(0)
            self.M2B.duty_u16(int(duty_right * self.right_bias)) 

    def stop(self):
        """Stop both motors."""
        self.M1A.duty_u16(0)
        self.M1B.duty_u16(0)
        self.M2A.duty_u16(0)
        self.M2B.duty_u16(0)
        print("Motors stopped.")