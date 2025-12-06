import machine
import time
import neopixel

# Wait for USB and peripherals to settle before anything else
# time.sleep(2)

# Speaker Data
speaker = machine.PWM(machine.Pin(22))
speaker.freq(8)
time.sleep(0.1)

class TimeoutError(Exception):
    pass

class Ultrasound():
    def __init__(self, trigger, echo, timeout=40000):
        self.t = trigger
        self.e = echo
        self.timeout = timeout

    def measure(self):
        # small pause to avoid timing lockups
        # time.sleep_ms(10)
        # create trigger pulse
        self.t.low()
        time.sleep_us(2)
        self.t.high()
        time.sleep_us(15)
        self.t.low()
        start_time = time.ticks_us()

        # wait for start of echo
        while self.e.value() == 0:
            signaloff = time.ticks_us()
            if time.ticks_diff(signaloff, start_time) >= self.timeout:
                raise TimeoutError

        # measure echo width
        start_time = time.ticks_us()
        while self.e.value() == 1:
            signalon = time.ticks_us()
            if time.ticks_diff(signalon, start_time) >= self.timeout:
                raise TimeoutError

        # compute width
        timepassed = signalon - signaloff

        # return distance
        return ((timepassed / 1000) * 16.7) + 1.19  # *17.4 + 3.28