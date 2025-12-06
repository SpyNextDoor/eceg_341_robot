from line_sensor_02 import LineReader
from motor import Motor
import time
from machine import Pin

# 1. --- Setup ---
# Instantiate with auto mode detection
lr = LineReader(pins=[Pin(0), Pin(1), Pin(2), Pin(3), Pin(4), Pin(5)], 
                positions=[-20, -12, -4, 4, 12, 20], 
                samples=40,
                mode="auto")  # Auto-detect line type
m = Motor()

# --- Adaptive Control Parameters ---
# These get updated based on detected mode
class ControlParams:
    def __init__(self):
        # Grey mode parameters (gentler control)
        self.grey_kp = -0.20
        self.grey_kd = -0.0008
        self.grey_base_velocity = 12
        self.grey_turn_velocity = 10
        
        # Black mode parameters (more aggressive)
        self.black_kp = -0.25
        self.black_kd = -0.001
        self.black_base_velocity = 15
        self.black_turn_velocity = 13
        
        # Current active parameters
        self.kp = self.grey_kp
        self.kd = self.grey_kd
        self.base_velocity = self.grey_base_velocity
        self.turn_velocity = self.grey_turn_velocity
        self.current_mode = "grey_mode"
    
    def update_for_mode(self, mode):
        """Update control parameters based on detected mode."""
        if mode == self.current_mode:
            return  # No change needed
        
        if mode == "grey_mode":
            self.kp = self.grey_kp
            self.kd = self.grey_kd
            self.base_velocity = self.grey_base_velocity
            self.turn_velocity = self.grey_turn_velocity
            print(f"Switched to GREY mode control (Kp={self.kp}, vel={self.base_velocity})")
        else:  # black_mode
            self.kp = self.black_kp
            self.kd = self.black_kd
            self.base_velocity = self.black_base_velocity
            self.turn_velocity = self.black_turn_velocity
            print(f"Switched to BLACK mode control (Kp={self.kp}, vel={self.base_velocity})")
        
        self.current_mode = mode

params = ControlParams()

# --- Loop Variables ---
last_offset = 0.0
last_time_us = time.ticks_us() - 10000  # Initialize in the past
lost_line_count = 0
MAX_LOST_COUNT = 10  # Stop if line lost for this many iterations
ERROR_THRESHOLD = 7  # When to reduce speed

print("Starting adaptive line follower...")
print("Will auto-detect grey/black lines and adjust control")

try:
    while True:
        # --- Calculate Time Delta ---
        current_time_us = time.ticks_us()
        dt_us = time.ticks_diff(current_time_us, last_time_us)
        dt_s = dt_us / 1000000.0
        last_time_us = current_time_us

        # --- Get Sensor Reading ---
        lr.update()
        
        # --- Adapt Control Parameters to Detected Mode ---
        detected_mode = lr.get_mode()
        params.update_for_mode(detected_mode)
        
        # --- Handle Line Detection ---
        offset = lr.get_offset()
        
        if offset is None:
            # Line lost - handle gracefully
            lost_line_count += 1
            if lost_line_count > MAX_LOST_COUNT:
                print(f"Line lost for {MAX_LOST_COUNT} iterations - stopping")
                m.stop()
                time.sleep(0.5)
                lost_line_count = 0
                # Continue searching
                continue
            else:
                # Briefly continue with last known offset
                offset = last_offset
                print(f"Line lost ({lost_line_count}/{MAX_LOST_COUNT}), using last offset")
        else:
            # Line found - reset lost counter
            if lost_line_count > 0:
                print("Line reacquired!")
            lost_line_count = 0
        
        # --- PD Control Logic ---
        error = offset
        
        # Adaptive velocity based on error magnitude
        if abs(error) > ERROR_THRESHOLD:
            velocity = params.turn_velocity  # Slow down for sharp turns
        else:
            velocity = params.base_velocity  # Full speed on straight sections
        
        # D-term (derivative)
        derivative = 0
        if dt_s > 0:
            derivative = (error - last_offset) / dt_s
        
        # PD Calculation with adaptive gains
        angular_velocity = (params.kp * error) + (params.kd * derivative)
        
        # Save current error for next iteration
        last_offset = error
        
        # --- Send Motor Commands ---
        m.drive(velocity, angular_velocity)
        
        # Optional: Debug output (comment out for performance)
        # print(f"Mode: {detected_mode}, Offset: {offset:.1f}, "
        #       f"Vel: {velocity}, AngVel: {angular_velocity:.2f}, "
        #       f"Contrast: {lr.get_contrast():.1f}")
        
        # Small delay to prevent overwhelming the system
        time.sleep_ms(5)

except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    m.stop()
    print("Motors stopped")