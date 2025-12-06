from line_sensor import LineReader
from ultrasound import Ultrasound, TimeoutError
from motor import Motor
import time
from machine import Pin

# 1. --- Setup ---
# Instantiate your classes
lr = LineReader(pins = [Pin(0), Pin(1), Pin(2), Pin(3), Pin(4), Pin(5)], 
                    positions = [-20, -12, -4, 4, 12, 20], samples = 40) # Assuming default pins/positions
m = Motor() # you might have different constructor values

ultrasound = Ultrasound(trigger = Pin(28, Pin.OUT), echo = Pin(7, Pin.IN))

# Set a base speed. 30 is a good start.
# velocity = 25

# # 3. --- Control Loop ---
# Add these variables *before* the loop
last_offset = 0.0
last_time_us = time.ticks_us() - 10000 # Initialize in the past

# Kp = -0.25 # Proportional gain
# Kd = -0.001 # Derivative gain (start with a small value!)

Kp = -0.2 # Proportional gain
Kd = -0.02 # Derivative gain (start with a small value!)

velocity = 0          # current speed (starts slow)
max_velocity = 30     # fastest allowed speed
min_velocity = 10     # slowest allowed speed
accel_rate = 3.5      # how quickly robot speeds up per loop
decel_rate = 6.5      # 8.5 how quickly robot slows down per loop

try:
    while True:
        # measure distance as robot goes
        try:
            distance = ultrasound.measure()
        except TimeoutError:
            distance = 999 # no echo → assume nothing is close

        if distance > 5:
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
            if abs(error) > 5:
                target_velocity = 10
            else:
                target_velocity = 25

            # 2. Smooth acceleration / deceleration
            if velocity < target_velocity:
                velocity += accel_rate      # speed up slowly
            elif velocity > target_velocity:
                velocity -= decel_rate      # slow down quickly

            # 3. Limit velocity to safe range
            velocity = max(min_velocity, min(max_velocity, velocity))

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

        else:
            m.stop()

finally:
    m.stop() # Always stop the motors