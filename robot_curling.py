# line_follow.py
from line_sensor import LineReader
from ultrasound import Ultrasound, TimeoutError
from motor import Motor
import neopixel
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

# Set a base speed. 30 is a good start.
# velocity = 25

# # 3. --- Control Loop ---
# Add these variables *before* the loop
last_offset = 0.0
last_time_us = time.ticks_us() - 10000 # Initialize in the past

Kp = -0.2 # Proportional gain
Kd = -0.02 # Derivative gain (start with a small value!)

# ----- INITIALIZATION -----
state = "FOLLOW_LINE"
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

    # ========== STATE: FOLLOW_LINE ==========
    if state == "FOLLOW_LINE":
        # PD control
        error = lr.offset

        # Speed logic
        if error is not None:
            velocity = 13 if abs(error) > 5 else 17
        else:
            velocity = 15

        # Derivative
        derivative = (error - last_offset) / dt_s if dt_s > 0 else 0
        last_offset = error

        angular_velocity = (Kp * error) + (Kd * derivative)

        m.drive(velocity, angular_velocity)

        # Stay in this state for 5 seconds
        if time.ticks_diff(time.ticks_ms(), state_entry_time) > 10000:
            m.stop()
            state = "PAUSE"
            state_entry_time = time.ticks_ms()

    # ========== STATE: PAUSE ==========
    elif state == "PAUSE":
        m.stop()

        # 1 second pause
        if time.ticks_diff(time.ticks_ms(), state_entry_time) > 1000:
            state = "FOWARD"
            state_entry_time = time.ticks_ms()

    # ========== STATE: FOWARD ==========
    elif state == "FOWARD":
        m.drive(40, 0)

        # 1 second pause
        if time.ticks_diff(time.ticks_ms(), state_entry_time) > 1500:
            state = "DRIVE_TO_TARGET"
             # Set the LED color
            np[0] = (0, 127, 255)
            np[1] = (0, 127, 255)
            np.write()
            state_entry_time = time.ticks_ms()


    # ========== STATE: DRIVE_TO_TARGET ==========
    elif state == "DRIVE_TO_TARGET":
        m.drive(15, 0)
        # Set the LED color
        # np[0] = (0, 127, 255)
        # np[1] = (0, 127, 255)
        # np.write()

        if distance <= 45: # for the competition put this back to 45
            m.stop()
             # Set the LED OFF
            np[0] = (0, 0, 0)
            np[1] = (0, 0, 0)
            np.write()
            state = "TARGET_REACHED"

    # ========== STATE: TARGET_REACHED ==========
    elif state == "TARGET_REACHED":
        m.stop()

        # Turn neopixel off
        # np[0] = (0, 127, 255)
        # np[1] = (0, 127, 255)
        # np.write()
        # You can add more behavior here
        pass
