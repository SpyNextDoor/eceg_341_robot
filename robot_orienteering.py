# line_follow.py
from line_sensor_02 import LineReader
from ultrasound import Ultrasound, TimeoutError
from motor import Motor
import neopixel
import math
import time
from machine import Pin

# 1. --- Setup ---
# Instantiate your classes
lr = LineReader(pins = [Pin(0), Pin(1), Pin(2), Pin(3), Pin(4), Pin(5)], 
                    positions = [-20, -12, -4, 4, 12, 20], samples = 40) # Assuming default pins/positions
m = Motor() # you might have different constructor values

NUM_PIXELS = 2
GP_NUM = 18
np = neopixel.NeoPixel(machine.Pin(GP_NUM), NUM_PIXELS)

ultrasound = Ultrasound(trigger = Pin(28, Pin.OUT), echo = Pin(7, Pin.IN))

def turn(angle_deg, speed=10, duration_ms=500):
    m.drive(speed, math.radians(angle_deg))
    time.sleep_ms(duration_ms)
    m.drive(0, 0)
    time.sleep_ms(200)

def forward(speed, duration):
    m.drive(speed, 0)
    time.sleep(duration)
    m.drive(0, 0)
    time.sleep_ms(200)

# # 3. --- Control Loop ---
# Add these variables *before* the loop
last_offset = 0.0
last_time_us = time.ticks_us() - 10000 # Initialize in the past

Kp = -0.2 # Proportional gain
Kd = -0.02 # Derivative gain (start with a small value!)

# ----- INITIALIZATION -----
state = "CENTER"
state_entry_time = time.ticks_ms()

while True:
    # ---------------------------------------------
    # Common timing update
    current_time_us = time.ticks_us()
    dt_us = time.ticks_diff(current_time_us, last_time_us)
    dt_s = dt_us / 1_000_000
    last_time_us = current_time_us
    # ---------------------------------------------

    # Read sensors
    try:
        distance = ultrasound.measure()

    except TimeoutError:
        distance = 999 # no echo → assume nothing is close

    lr.update()
    error = lr.get_offset()

    # ========== STATE: CENTER ==========
    if state == "CENTER":
        # PD control
        error = lr.offset

        # Speed logic
        velocity = 13 if abs(error) > 5 else 17

        # Derivative
        derivative = (error - last_offset) / dt_s if dt_s > 0 else 0
        last_offset = error

        angular_velocity = Kp * error + Kd * derivative

        m.drive(velocity, angular_velocity)

        # Stay in this state for 5 seconds
        if time.ticks_diff(time.ticks_ms(), state_entry_time) > 3600:
            m.stop()
            state = "STATION_1"
            state_entry_time = time.ticks_ms()

    # ========== STATE: STATION_1 ==========
    elif state == "STATION_1":
        m.stop()
        np[0] = (255, 0, 0)
        np[1] = (255, 0, 0)
        np.write()
        time.sleep(0.3)
        np[0] = (0, 0, 0)
        np.write()
        time.sleep(0.5)

        turn(360)

        # PD control
        error = lr.offset

        # Speed logic
        velocity = 13 if abs(error) > 5 else 17

        # Derivative
        derivative = (error - last_offset) / dt_s if dt_s > 0 else 0
        last_offset = error

        angular_velocity = Kp * error + Kd * derivative

        m.drive(velocity, angular_velocity)

        # Stay in this state for 5 seconds
        if time.ticks_diff(time.ticks_ms(), state_entry_time) > 3600:
            m.stop()
            state = "CENTER"
            state_entry_time = time.ticks_ms()
        

    #     # 1 second pause
    #     if time.ticks_diff(time.ticks_ms(), state_entry_time) > 2000:
    #         state = "STOP"

    # elif state == "STOP":
    #     m.stop()

    #     # Turn neopixel off
    #     np[0] = (255, 0, 0)
    #     np[1] = (255, 0, 0)
    #     np.write()
    #     # You can add more behavior here
    #     pass
