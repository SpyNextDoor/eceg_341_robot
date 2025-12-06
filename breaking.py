# line_follow.py
from line_sensor import LineReader
from motor import Motor
import time
import neopixel
import math
from machine import Pin, PWM

NUM_PIXELS = 2
GP_NUM = 18

# 1. --- Setup ---
# Instantiate your classes
lr = LineReader(pins = [Pin(0), Pin(1), Pin(2), Pin(3), Pin(4), Pin(5)], 
                    positions = [-20, -12, -4, 4, 12, 20], samples = 40) # Assuming default pins/positions
m = Motor() # you might have different constructor values

# Neopixel
np = neopixel.NeoPixel(Pin(GP_NUM), NUM_PIXELS)

#speaker 
speaker = PWM(Pin(22))

# musical notes
note_data = [
    {"note": "C4", "frequency": 261.63, "color_name": "Red", "rgb": (255, 0, 0), "distance_cm": 40},
    {"note": "C#4/Db4", "frequency": 277.18, "color_name": "Orange-Red", "rgb": (255, 69, 0), "distance_cm": 35},
    {"note": "D4", "frequency": 293.66, "color_name": "Orange", "rgb": (255, 140, 0), "distance_cm": 30},
    {"note": "D#4/Eb4", "frequency": 311.13, "color_name": "Yellow", "rgb": (255, 255, 0), "distance_cm": 25},
    {"note": "E4", "frequency": 329.63, "color_name": "Chartreuse", "rgb": (127, 255, 0), "distance_cm": 20},
    {"note": "F4", "frequency": 349.23, "color_name": "Green", "rgb": (0, 255, 0), "distance_cm": 15},
    {"note": "F#4/Gb4", "frequency": 369.99, "color_name": "Spring Green", "rgb": (0, 255, 127), "distance_cm": 10},
    {"note": "G4", "frequency": 392.00, "color_name": "Cyan", "rgb": (0, 255, 255), "distance_cm": 8},
    {"note": "G#4/Ab4", "frequency": 415.30, "color_name": "Azure", "rgb": (0, 127, 255), "distance_cm": 6},
    {"note": "A4", "frequency": 440.00, "color_name": "Blue", "rgb": (0, 0, 255), "distance_cm": 5},
    {"note": "A#4/Bb4", "frequency": 466.16, "color_name": "Violet", "rgb": (139, 0, 255), "distance_cm": 4},
    {"note": "B4", "frequency": 493.88, "color_name": "Magenta", "rgb": (255, 0, 255), "distance_cm": 3}
]

# foward, turn_right, reverse, turn_left

state = "foward"
velocity = 20
state_entry_time = time.ticks_ms()

def blink():
    global state
    while state == "turn_right" or state == "turn_left":
        if state == "turn_right":
            np[0] = (255, 0, 0)
            np.write()
            time.sleep(0.3)
            np[0] = (0, 0, 0)
            np.write()
            time.sleep(0.3)
        elif state == "turn_left":
            np[1] = (0, 255, 0)
            np.write()
            time.sleep(0.3)
            np[0] = (0, 0, 0)
            np.write()
            time.sleep(0.3)

        if state != "turn_right" or state != "turn_left":
            break

try:

    while True:
        if state == "foward":
            m.drive(velocity, 0)
            np[0] = (0, 127, 255)
            np[1] = (0, 127, 255)
            np.write()

            speaker.freq(int(261.63))
            speaker.duty_u16(32768)
            
            if time.ticks_diff(time.ticks_ms(), state_entry_time) > 1000:
                m.stop()
                time.sleep(1)
                state = "turn_right"
                state_entry_time = time.ticks_ms()

        elif state == "turn_right":
            m.drive(velocity, math.radians(90))
            speaker.freq(int(293.66))
            speaker.duty_u16(32768)

            if time.ticks_diff(time.ticks_ms(), state_entry_time) > 1000:
                m.stop()
                state = "reverse"
                state_entry_time = time.ticks_ms()

        elif state == "reverse":
            m.drive(-velocity, 0)
            np[0] = (255, 0, 255)
            np[1] = (255, 0, 255)
            np.write()

            speaker.freq(int(329.63))
            speaker.duty_u16(32768)
          

            if time.ticks_diff(time.ticks_ms(), state_entry_time) > 1000:
                m.stop()
                time.sleep(1)
                state = "turn_left"
                state_entry_time = time.ticks_ms()

        elif state == "turn_left":
            m.drive(velocity, math.radians(-90))

            speaker.freq(int(293.66))
            speaker.duty_u16(32768)
           
            blink()

            if time.ticks_diff(time.ticks_ms(), state_entry_time) > 1000:
                m.stop()
                
                speaker.duty_u16(0)
                state = "stop"
                state_entry_time = time.ticks_ms()

        elif state == "stop":
            m.stop()
            np[0] = (0, 0, 0)
            np[1] = (0, 0, 0)
            np.write()

            speaker.duty_u16(0)
            
            time.sleep(1)

            


finally:
    m.stop()


# # # 3. --- Control Loop ---
# # Add these variables *before* the loop
# last_offset = 0.0
# last_time_us = time.ticks_us() - 10000 # Initialize in the past

# Kp = -0.25 # Proportional gain
# Kd = -0.001 # Derivative gain (start with a small value!)

# try:
#     while True:
#         # --- PD Control Logic ---
        
#         # Calculate dt (time delta)
#         current_time_us = time.ticks_us()
#         dt_us = time.ticks_diff(current_time_us, last_time_us)
#         dt_s = dt_us / 1000000.0 # convert to seconds
#         last_time_us = current_time_us

#         # Get sensor reading
#         lr.update()
        
#         # P-term
#         error = lr.offset
#         if abs(error) > 5:
#             velocity = 13
#         else:
#             velocity = 40
        
#         # D-term
#         # Note: We must check dt_s to avoid ZeroDivisionError
#         # This also handles the first loop
#         derivative = 0
#         if dt_s > 0:
#             derivative = (error - last_offset) / dt_s
        
#         # PD Calculation
#         angular_velocity = (Kp * error) + (Kd * derivative)
        
#         # Save current error for next loop
#         last_offset = error
        
#         # --- END CONTROL LOGIC ---
        
#         # Send command to motors
#         m.drive(velocity, angular_velocity)
        
#         # ... (rest of your loop with confidence check) ...

# finally:
#     m.stop() # Always stop the motors