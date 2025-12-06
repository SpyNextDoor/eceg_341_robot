# line_follow.py
from line_sensor import LineReader
from motor import Motor
import time
from machine import Pin

# 1. --- Setup ---
# Instantiate your classes
lr = LineReader(pins = [Pin(0), Pin(1), Pin(2), Pin(3), Pin(4), Pin(5)], 
                    positions = [-20, -12, -4, 4, 12, 20], samples = 40) # Assuming default pins/positions
m = Motor() # you might have different constructor values

# # 3. --- Control Loop ---
# Add these variables *before* the loop
last_offset = 0.0
last_time_us = time.ticks_us() - 10000 # Initialize in the past

Kp = -0.25 # Proportional gain
Kd = -0.001 # Derivative gain (start with a small value!)

try:
    while True:
        # --- PD Control Logic ---
        
        # Calculate dt (time delta)
        current_time_us = time.ticks_us()
        dt_us = time.ticks_diff(current_time_us, last_time_us)
        dt_s = dt_us / 1000000.0 # convert to seconds
        last_time_us = current_time_us

        # Get sensor reading
        lr.update()
        
        # P-term
        error = lr.offset
        if abs(error) > 7:
            velocity = 13
        else:
            velocity = 15
        
        # D-term
        # Note: We must check dt_s to avoid ZeroDivisionError
        # This also handles the first loop
        derivative = 0
        if dt_s > 0:
            derivative = (error - last_offset) / dt_s
        
        # PD Calculation
        angular_velocity = (Kp * error) + (Kd * derivative)
        
        # Save current error for next loop
        last_offset = error
        
        # --- END CONTROL LOGIC ---
        
        # Send command to motors
        m.drive(velocity, angular_velocity)
        
        # ... (rest of your loop with confidence check) ...

finally:
    m.stop() # Always stop the motors


