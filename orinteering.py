# orienteering.py
from line_sensor_02 import LineReader
from ultrasound import Ultrasound, TimeoutError
from motor import Motor
import neopixel
import math
import time
from machine import Pin

# ========== SETUP ==========
lr = LineReader(pins=[Pin(0), Pin(1), Pin(2), Pin(3), Pin(4), Pin(5)], 
                positions=[-20, -12, -4, 4, 12, 20], 
                samples=40,
                mode="auto")  # Auto-detect line type

m = Motor()

# NeoPixel setup
NUM_PIXELS = 2
GP_NUM = 18
np = neopixel.NeoPixel(Pin(GP_NUM), NUM_PIXELS)

# Ultrasound setup
ultrasound = Ultrasound(trigger=Pin(28, Pin.OUT), echo=Pin(7, Pin.IN))

# ========== ADAPTIVE CONTROL PARAMETERS ==========
class ControlParams:
    def __init__(self):
        # Grey mode
        self.grey_kp = -0.20
        self.grey_kd = -0.0008
        self.grey_velocity = 12
        self.grey_turn_velocity = 10
        
        # Black mode
        self.black_kp = -0.25
        self.black_kd = -0.001
        self.black_velocity = 15
        self.black_turn_velocity = 13
        
        # Active params
        self.kp = self.grey_kp
        self.kd = self.grey_kd
        self.velocity = self.grey_velocity
        self.turn_velocity = self.grey_turn_velocity
    
    def update_for_mode(self, mode):
        if mode == "grey_mode":
            self.kp = self.grey_kp
            self.kd = self.grey_kd
            self.velocity = self.grey_velocity
            self.turn_velocity = self.grey_turn_velocity
        else:
            self.kp = self.black_kp
            self.kd = self.black_kd
            self.velocity = self.black_velocity
            self.turn_velocity = self.black_turn_velocity

params = ControlParams()

# ========== HELPER FUNCTIONS ==========
def set_led(color):
    """Set LED color. color = (R, G, B) tuple or None for off."""
    if color is None:
        np[0] = (0, 0, 0)
        np[1] = (0, 0, 0)
    else:
        np[0] = color
        np[1] = color
    np.write()

def blink_led(color, times=3, duration_ms=200):
    """Blink LED to indicate station reached."""
    for _ in range(times):
        set_led(color)
        time.sleep_ms(duration_ms)
        set_led(None)
        time.sleep_ms(duration_ms)

def turn_around():
    """Execute a 180-degree turn."""
    print("Turning around...")
    m.drive(0, math.radians(180))  # Turn in place
    time.sleep_ms(1000)  # Adjust timing as needed
    m.stop()
    time.sleep_ms(200)

def pd_control(error, last_error, dt_s):
    """Calculate PD control output."""
    derivative = 0
    if dt_s > 0:
        derivative = (error - last_error) / dt_s
    
    angular_velocity = (params.kp * error) + (params.kd * derivative)
    return angular_velocity

def follow_line():
    """Execute one iteration of line following. Returns (error, velocity)."""
    lr.update()
    
    # Adapt to detected mode
    detected_mode = lr.get_mode()
    params.update_for_mode(detected_mode)
    
    offset = lr.get_offset()
    
    # Handle line loss
    if offset is None:
        print("Warning: Line lost!")
        return None, 0
    
    # Adaptive velocity
    if abs(offset) > 7:
        velocity = params.turn_velocity
    else:
        velocity = params.velocity
    
    return offset, velocity

# ========== STATE MACHINE CONFIGURATION ==========
# Station colors for LED feedback
STATION_COLORS = [
    (255, 0, 0),    # Station 1: Red
    (0, 255, 0),    # Station 2: Green
    (0, 0, 255),    # Station 3: Blue
    (255, 255, 0)   # Station 4: Yellow
]

# Detection parameters
STATION_DISTANCE_CM = 15  # Distance to detect station marker
STATION_STOP_TIME_MS = 1000  # How long to stop at station

# ========== STATE MACHINE ==========
state = "FOLLOWING_TO_STATION_1"
current_station = 0  # 0-3 for stations 1-4
stations_visited = 0
returning_home = False

# PD control variables
last_offset = 0.0
last_time_us = time.ticks_us() - 10000

# State timing
state_entry_time = time.ticks_ms()

print("Starting Orienteering Challenge!")
print(f"Target: Visit {len(STATION_COLORS)} stations and return home")

try:
    while True:
        # ========== TIMING UPDATE ==========
        current_time_us = time.ticks_us()
        dt_us = time.ticks_diff(current_time_us, last_time_us)
        dt_s = dt_us / 1_000_000
        last_time_us = current_time_us
        
        # ========== SENSOR READINGS ==========
        try:
            distance = ultrasound.measure()
        except TimeoutError:
            distance = 999  # No obstacle detected
        
        # ========== STATE: FOLLOWING TO STATION ==========
        if state.startswith("FOLLOWING_TO_STATION"):
            offset, velocity = follow_line()
            
            if offset is None:
                # Line lost - stop and search
                m.stop()
                print("Lost line while following!")
                continue
            
            # PD control
            angular_velocity = pd_control(offset, last_offset, dt_s)
            last_offset = offset
            
            # Drive
            m.drive(velocity, angular_velocity)
            
            # Check for station marker
            if distance < STATION_DISTANCE_CM:
                print(f"Station {current_station + 1} detected! (distance: {distance}cm)")
                m.stop()
                state = "AT_STATION"
                state_entry_time = time.ticks_ms()
        
        # ========== STATE: AT STATION ==========
        elif state == "AT_STATION":
            m.stop()
            
            # Light up LED for current station
            color = STATION_COLORS[current_station]
            blink_led(color, times=3)
            
            print(f"Completed station {current_station + 1}/{len(STATION_COLORS)}")
            stations_visited += 1
            
            # Decide next action
            if stations_visited >= len(STATION_COLORS):
                print("All stations visited! Returning home...")
                state = "TURN_AROUND"
                returning_home = True
            else:
                print(f"Continuing to station {stations_visited + 1}...")
                current_station += 1
                state = f"FOLLOWING_TO_STATION_{current_station + 1}"
            
            state_entry_time = time.ticks_ms()
        
        # ========== STATE: TURN AROUND ==========
        elif state == "TURN_AROUND":
            turn_around()
            state = "RETURNING_HOME"
            state_entry_time = time.ticks_ms()
        
        # ========== STATE: RETURNING HOME ==========
        elif state == "RETURNING_HOME":
            offset, velocity = follow_line()
            
            if offset is None:
                m.stop()
                print("Lost line while returning!")
                continue
            
            # PD control
            angular_velocity = pd_control(offset, last_offset, dt_s)
            last_offset = offset
            
            # Drive
            m.drive(velocity, angular_velocity)
            
            # Check for home marker (same distance detection)
            if distance < STATION_DISTANCE_CM:
                print("Home marker detected!")
                m.stop()
                state = "FINISHED"
                state_entry_time = time.ticks_ms()
        
        # ========== STATE: FINISHED ==========
        elif state == "FINISHED":
            m.stop()
            print("=== CHALLENGE COMPLETE! ===")
            blink_led((255, 255, 255), times=5)  # White celebration
            break
        
        # Small delay
        time.sleep_ms(5)

except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    m.stop()
    set_led(None)
    print("Motors stopped, LEDs off")