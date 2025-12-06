
import time
from machine import Pin
import math

class LineReader:
    def __init__(self, pins=[0,1,2,3,4,5], positions=[-20, -12, -4, 4, 12, 20], 
                 samples=40, mode="auto"):
        self.pins = pins
        self.positions = positions
        self.offset = 0.0
        self.samples = samples
        self.darkness = 0.0
        self.confidence = 0.0
        self.mode = mode  # Can be "auto", "grey_mode", or "black_mode"
        self.detected_mode = "grey_mode"  # Current detected mode
        
        # Adaptive thresholds
        self.grey_threshold = 15
        self.black_threshold = 100
        self.mode_switch_threshold = 50  # Contrast level to decide mode
        
        # History for mode detection
        self.contrast_history = []
        self.history_size = 10

    def _reflective_sampling(self, delay_us):
        """Perform reflective sampling and return decay times for each sensor."""
        # charge capacitance
        for pin in self.pins:
            pin.init(Pin.OUT, value=1)

        time.sleep_us(10)

        for pin in self.pins:
            # change to input
            pin.init(Pin.IN, pull=None)

        counts = [0] * len(self.pins)
        for i in range(self.samples):
            # wait one sample period
            time.sleep_us(delay_us)
            for j, pin in enumerate(self.pins):
                # count the number of 1's
                counts[j] += pin.value()    

        # the pulse width is the number of 1's 
        # detected times the delay
        decay_times = [delay_us * c for c in counts]
        
        return decay_times

    def _calculate_confidence(self, decay_times, mean_val):
        """Calculate confidence based on variance of decay times."""
        # Compute variance manually (MicroPython has no statistics module)
        variance = sum((x - mean_val) ** 2 for x in decay_times) / len(decay_times)
        
        # Higher variance = more confident we see a line (good contrast)
        # Tune lam to adjust sensitivity
        lam = 0.0001
        confidence = 1 - math.exp(-lam * variance)
        
        return confidence

    def _detect_mode(self, contrast):
        """Automatically detect if we're looking at grey or black lines."""
        if self.mode != "auto":
            # Manual mode override
            return self.mode
        
        # Add current contrast to history
        self.contrast_history.append(contrast)
        if len(self.contrast_history) > self.history_size:
            self.contrast_history.pop(0)
        
        # Need enough samples to make decision
        if len(self.contrast_history) < 3:
            return self.detected_mode  # Keep current mode
        
        # Calculate average contrast over recent history
        avg_contrast = sum(self.contrast_history) / len(self.contrast_history)
        
        # Decide mode based on average contrast level
        if avg_contrast > self.mode_switch_threshold:
            new_mode = "black_mode"
        else:
            new_mode = "grey_mode"
        
        # Only switch if mode changed
        if new_mode != self.detected_mode:
            print(f"Mode switched: {self.detected_mode} -> {new_mode} (contrast: {avg_contrast:.1f})")
            self.detected_mode = new_mode
        
        return self.detected_mode

    def update(self, delay_us=15):
        """Update sensor readings and calculate line position."""
        decay_times = self._reflective_sampling(delay_us)

        # Average decay time across all sensors (overall darkness)
        mean_val = sum(decay_times) / len(self.pins)
        self.darkness = mean_val
        
        # Calculate confidence based on variance
        self.confidence = self._calculate_confidence(decay_times, mean_val)

        # Normalize decay times relative to minimum
        min_val = min(decay_times)
        subtract_min = [dt - min_val for dt in decay_times]
        max_val = max(subtract_min)
        
        # Auto-detect mode based on contrast
        current_mode = self._detect_mode(max_val)
        
        # Set threshold based on detected mode
        if current_mode == "grey_mode":
            threshold = self.grey_threshold
        else:
            threshold = self.black_threshold

        # Check if we have enough contrast to detect a line
        if max_val < threshold:
            # Insufficient contrast - no line detected
            self.offset = None
            return
        
        # Normalize to sum to 1.0 for weighted average
        total = sum(subtract_min)
        if total == 0:
            # All sensors read the same - no line
            self.offset = None
            return
            
        normalized_decay = [x / total for x in subtract_min]
        
        # Calculate weighted position (offset from center)
        self.offset = sum(normalized_decay[i] * self.positions[i] 
                         for i in range(len(self.positions)))
            
    def get_offset(self):
        """Get the calculated line offset (None if no line detected)."""
        return self.offset
    
    def get_darkness(self):
        """Get the average darkness reading across all sensors."""
        return self.darkness
    
    def get_confidence(self):
        """Get the confidence score (0-1) based on reading variance."""
        return self.confidence
    
    def get_mode(self):
        """Get the current operating mode."""
        return self.detected_mode if self.mode == "auto" else self.mode
    
    def set_mode(self, mode):
        """Manually set mode: 'auto', 'grey_mode', or 'black_mode'."""
        if mode in ["auto", "grey_mode", "black_mode"]:
            self.mode = mode
            print(f"Mode set to: {mode}")
        else:
            print(f"Invalid mode: {mode}")
    
    def get_contrast(self):
        """Get current contrast level (max - min of readings)."""
        if len(self.contrast_history) > 0:
            return self.contrast_history[-1]
        return 0
    
# if __name__ == "__main__":
#     # test_sensor.py
#     lr = LineReader(pins=[Pin(0), Pin(1), Pin(2), Pin(3), Pin(4), Pin(5)], 
#                     mode="auto")  # Auto-detect mode

#     print("Testing LineReader with auto mode detection")
#     print("Move sensor over different surfaces to see mode changes\n")

#     while True:
#         lr.update()
#         print(f"Offset: {lr.get_offset()}, "
#               f"Darkness: {lr.get_darkness():.1f}, "
#               f"Confidence: {lr.get_confidence():.3f}, "
#               f"Mode: {lr.get_mode()}, "
#               f"Contrast: {lr.get_contrast():.1f}")
#         time.sleep(0.2)


