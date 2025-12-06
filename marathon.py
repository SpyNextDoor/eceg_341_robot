from line_sensor import LineReader
from ultrasound import Ultrasound, TimeoutError
from motor import Motor
import time
import neopixel
from machine import Pin
import math

# ----------- SETUP -----------
lr = LineReader(
    pins=[Pin(0), Pin(1), Pin(2), Pin(3), Pin(4), Pin(5)],
    positions=[-20, -12, -4, 4, 12, 20],
    samples=40
)

m = Motor()
ultrasound = Ultrasound(trigger=Pin(28, Pin.OUT), echo=Pin(7, Pin.IN))

NUM_PIXELS = 2
GP_NUM = 18

# Neopixel
np = neopixel.NeoPixel(Pin(GP_NUM), NUM_PIXELS)

# PD control initialization
last_offset = 0.0
last_time_us = time.ticks_us() - 10000

Kp = -0.2
Kd = -0.02

count = 0

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

# FSM
state = "FOLLOW_LINE"
state_entry_time = time.ticks_ms()

# ----------- MAIN LOOP -----------
while True:
    # Time step
    current_time_us = time.ticks_us()
    dt_us = time.ticks_diff(current_time_us, last_time_us)
    dt_s = dt_us / 1_000_000
    last_time_us = current_time_us

    # Read sensors
    try:
        distance = ultrasound.measure()

    except TimeoutError:
        distance = 999 # no echo → assume nothing is close
    
    lr.update()

    # =====================================================
    # ===============  FOLLOW LINE STATE  ==================
    # =====================================================
    if state == "FOLLOW_LINE":

        error = lr.offset

        # Adjust speed depending on error
        velocity = 13 if abs(error) > 5 else 17

        # PD control
        derivative = (error - last_offset) / dt_s if dt_s > 0 else 0
        last_offset = error
        angular_velocity = Kp * error + Kd * derivative

        m.drive(velocity, angular_velocity)

        # Detect obstacle
        if distance < 15:
            m.stop()
            time.sleep(0.5)
            if count > 0:
                state = "OBSTACLE_02"
            else:
                state = "OBSTACLE_01"
            state_entry_time = time.ticks_ms()

    # =====================================================
    # =================  OBSTACLE 1 STATE  ================
    # =====================================================

    elif state == "OBSTACLE_01":
        print("Entered Obs 1")
        count += 1

        np[0] = (255, 0, 0) 
        np[1] = (255, 0, 0) 
        np.write()

        # Turn 60 degrees left
        turn(-90)  # negative for left turn
    
        
        # Move to curve search state
        state = "CURVE_SEARCH"
        state_entry_time = time.ticks_ms()

        # turn(150) # right
        # forward(25, 1)
        # turn(-180) # left
        # forward(25, 1)
        # turn(-200) # left
        # forward(25, 1.5)
        # turn(-190) # left
        # forward(15, 1.5)

        np[0] = (0, 0, 0) 
        np[1] = (0, 0, 0) 
        np.write()

        # state = "FIND_LINE"
        # state_entry_time = time.ticks_ms()

    # # =====================================================
# # ==============  CURVE SEARCH STATE  =================
# # =====================================================

    elif state == "CURVE_SEARCH":
        print("Entered curve search")
        lr.update()
        
        # Drive in a gentle arc (forward with slight right curve to compensate for left turn)
        m.drive(18, math.radians(20))  # move forward-right in an arc
        time.sleep(1.5)
        
        # Check if line is detected
        if abs(lr.offset) < 15:
            m.stop()
            time.sleep(0.1)
            turn(90)
            state = "FOLLOW_LINE"
            print("exit curve search")
            continue
        
        # Safety timeout - if line not found after 5 seconds, try recovery
        if time.ticks_diff(time.ticks_ms(), state_entry_time) > 5000:
            state = "FIND_LINE"
            state_entry_time = time.ticks_ms()

        # # =====================================================
        # # =================  FIND LINE STATE  =================
        # # =====================================================

    elif state == "FIND_LINE":

        lr.update()
        # 2. Drive forward
        # forward(10, 2)

        # If line is detected strongly enough, go back to FOLLOW_LINE
        if abs(lr.offset) < 15:
            m.stop()
            time.sleep(0.1)
            state = "FOLLOW_LINE"
            continue

        # Otherwise rotate slowly to search for the line
        # (You can reverse direction if needed)
        m.drive(8, math.radians(-40))  # slow rotation left

        # Safety timeout (so it doesn't spin forever)
        if time.ticks_diff(time.ticks_ms(), state_entry_time) > 4000:
            # Try the opposite direction
            m.drive(8, math.radians(40))
            time.sleep(1)
            state_entry_time = time.ticks_ms()

    
    # # =====================================================
    # # =================  OBSTACLE 2 STATE  ================
    # # =====================================================

    elif state == "OBSTACLE_02":
        count = 0

        np[0] = (0, 255, 0) 
        np[1] = (0, 255, 0) 
        np.write()

        turn(200) #right
        forward(15, 2)
        turn(-120) # left
        forward(15, 1.2)

        lr.update()
        
        # Drive in a gentle arc (forward with slight left curve to compensate for left turn)
        m.drive(10, math.radians(-40))  # move forward-left in an arc
        time.sleep(.9)

        m.drive(10, math.radians(90))
        
        
        # Check if line is detected
        if abs(lr.offset) < 15:
            m.stop()
            time.sleep(0.1)
            turn(80)
            state = "FOLLOW_LINE"
            print("exit curve search")
            continue
        
        # Safety timeout - if line not found after 5 seconds, try recovery
        if time.ticks_diff(time.ticks_ms(), state_entry_time) > 5000:
            state = "FIND_LINE"
            state_entry_time = time.ticks_ms()


        # state = "FIND_LINE"
        # state_entry_time = time.ticks_ms()