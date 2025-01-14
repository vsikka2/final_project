import time
from picar_4wd.servo import Servo
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin

class Ultrasonic():
    ANGLE_RANGE = 180
    STEP = 18

    def __init__(self, trig, echo, timeout=0.01):
        self.timeout = timeout
        self.trig = trig
        self.echo = echo
        # Init Servo
        self.servo = Servo(PWM("P0"), offset=10)
        self.angle_distance = [0,0]
        self.current_angle = 0
        self.max_angle = self.ANGLE_RANGE/2
        self.min_angle = -self.ANGLE_RANGE/2
        self.scan_list = []

    def get_distance(self):
        self.trig.low()
        time.sleep(0.01)
        self.trig.high()
        time.sleep(0.000015)
        self.trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while self.echo.value()==0:
            pulse_start = time.time()
            if pulse_start - timeout_start > self.timeout:
                return -1
        while self.echo.value()==1:
            pulse_end = time.time()
            if pulse_end - timeout_start > self.timeout:
                return -2
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        #print(cm)
        return cm
