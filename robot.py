import pigpio
import sys
import math
import time
from map import Map, Intersection
from datetime import datetime, timedelta
from typing import List, Tuple
from functools import reduce

# first element is the left motor; second is the right.
# first element of each tuple is the negative terminal; second is positive
MOTOR_PINS = [(7, 8), (5, 6)]

# colors correspond to the color of the jumper cable connected to each LED
LED_PINS = {
    18: 'right',
    15: 'middle',
    14: 'left',
}

# Cardinal directions
N = 0
W = 1
S = 2
E = 3

# Directions
F = 0   # forwards
L = 1   # left
B = 2   # backwards
R = 3   # right

# 90 degree turns
DEGREE90_PWM = 0.685
DEGREE90_WAIT = 0.89

def route_to_directions(route: List[Intersection], heading: int):
    diff_to_dir = {
        (0, 1):     F,
        (-1, 0):    L,
        (0, -1):    B,
        (1, 0):     R,
    }

    curr_heading = heading
    dirs = []
    for j in range(len(route)-1):
        direction = diff_to_dir[route[j+1]-route[j]]
        dirs.append(
            (
                direction - curr_heading
            ) % 4
        )
        print(curr_heading ,direction)
        print(dirs)
        curr_heading = direction

    return dirs



def duty_to_pwm(
    motor_number: int,
    dutycycle: float
) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    '''
    converts dutycycle to pwm

    Parameters
    ----------
    motor_number : int
        0 -> for left motor, 1 -> for right motor; errors if anything else
    dutycycle : float
        dutycycle, between -1.0 and 1.0

    Returns
    -------
    Tuple[Tuple[int, int], Tuple[int, int]]
        two tuples: the first is the (motor_pin, pwm) for the pin
        of the motor being set and the second is (other_motor_pin, 0)
    '''
    magnitude = abs(round(dutycycle*255))
    if magnitude > 255:
        # max pwm is 255
        magnitude = 255

    # negative dutycycle -> negative pin, positive -> positive pin
    pin_to_set = 0 if dutycycle < 0 else 1
    other_pin = 0 if pin_to_set == 1 else 1

    return ((MOTOR_PINS[motor_number][pin_to_set], magnitude),
            (MOTOR_PINS[motor_number][other_pin], 0))

class EEBot:
    '''eebot instance!

    Attributes
    ----------
    io : pigpio.pi
        pi I/O instance with which we can interact with our little eebot
    motors: List[int]
        GPIO pins that the motor leads are connected to;
        looking at the bot from the back, the pin connections in order
        are expected to be: [LEFT_NEG, LEFT_POS, RIGHT_NEG, RIGHT_POS]
    LED_detectors : List[int]
        GPIO pins that the LED/phototransistor detectors are connected to
    L : float
        distance from the rear axle to the caster wheel in meters
    d : float
        distance between the two wheels in meters

    Methods
    -------
    shutdown()
        stops robot
    detectors_status()
        prints the status of each LED detector
    set_pwm(leftdutycycle, rightdutycycle)
        sets pwm of left and right motors
    '''

    def __init__(
        self,
        
        L: float = 0.103,
        d: float = 0.131
    ):
        '''initializes eebot

        Parameters
        ----------
        motors : List[int]
            GPIO
        LED_detectors : List[int]
            GPIO pins that the LED/phototransistor detectors are connected to
        '''
        
        self.motors = [pin for motor in MOTOR_PINS for pin in motor]
        self.LED_detectors = list(LED_PINS.keys())
        self.L = L
        self.d = d

        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)

        for pin in self.motors:
            # Set up the four pins as output (commanding the motors).
            self.io.set_mode(pin, pigpio.OUTPUT)

            # Prepare the PWM.  The range gives the maximum value for 100%
            # duty cycle, using integer commands (1 up to max).
            self.io.set_PWM_range(pin, 255)

            # Set the PWM frequency to 1000Hz.  You could try 500Hz or 2000Hz
            # to see whether there is a difference?
            self.io.set_PWM_frequency(pin, 1000)

            # Clear all pins, just in case.
            self.io.set_PWM_dutycycle(pin, 0)

        for pin in self.LED_detectors:
            # setup four LED pins as input
            self.io.set_mode(pin, pigpio.INPUT)

    def shutdown(self):
        '''stops robot'''
        print("Turning off...")

        for pin in self.motors:
            self.io.set_PWM_dutycycle(pin, 0)

        self.io.stop()


    def detectors_status(self):
        '''prints status of each LED detector'''
        for pin in self.LED_detectors:
            print(LED_PINS[pin], self.io.read(pin))
        print('----')

    def set_pwm(self, leftdutycycle: float, rightdutycycle: float):
        '''
        sets the PWM for both motors

        Parameters
        ----------
        leftdutycycle
            dutycycle for left motor, between -1.0 and 1.0
        rightdutycycle
            dutycycle for left motor, between -1.0 and 1.0
        '''
        for motor_number, dutycycle in enumerate([leftdutycycle, rightdutycycle]):
            motor_pwm = duty_to_pwm(motor_number, dutycycle)

            self.io.set_PWM_dutycycle(*motor_pwm[0])
            self.io.set_PWM_dutycycle(*motor_pwm[1])

    def set(self, vel_nominal: float, steering_angle: float):
        '''
        sets the nominal velocity and steering angle

        Parameters
        ----------
        vel_nominal : float
            nominal velocity in m/s
        steering_angle : float
            steering angle in degrees
        '''
        theta = steering_angle * math.pi / 180

        vel_left = vel_nominal * (math.cos(theta) -
                                  self.d/self.L * math.sin(theta))
        vel_right = vel_nominal * (math.cos(theta) +
                                   self.d/self.L * math.sin(theta))

        # TODO: FIX ME!!!!!
        k = 1.5
        fric = .34 if vel_nominal > 0 else -.34

        if vel_nominal == 0:
            self.set_pwm(0,0)
        else: 
            self.set_pwm(
                k*vel_left + fric,
                k*vel_right + fric
            )
    def turn(self, value):
        turn = None
        #left turn
        if (value == 1 or value == -3):
            self.left_inplace()
        #180 turn
        elif (value == 2 or value == -2):
            self.backwards_inplace()
        #right turn
        elif (value == 3 or value == -1):
            self.right_inplace()
        #else no turn
        else:
            return

    def find_line(self, spin_right: bool) -> bool:
        sign = -1 if spin_right else 1
        self.set_pwm(-sign*0.62, sign*0.62)

        start_time = datetime.now()
        rml = [0, 0, 0]
        while rml[1] != 1:
            rml = [self.io.read(pin) for pin in self.LED_detectors]
            time_delta = datetime.now() - start_time
            if time_delta.seconds + time_delta.microseconds * 1e-6 >= 2.5:
                self.set_pwm(0, 0)
                return False
        self.set_pwm(0, 0)
        return True

    def left_inplace(self):
        self.set_pwm(-0.7, 0.7)
        time.sleep(0.80)

    def right_inplace(self):
        self.set_pwm(0.7, -0.7)
        time.sleep(0.77)

    def backwards_inplace(self):
        self.set_pwm(-0.7, 0.7)
        time.sleep(2*0.86)

    def check_intersection(self):
        pins = self.LED_detectors
        #forward, left, backward, right
        streets = [False, False, True, False]
        #go forward a bit
        for i in range(2000):
            self.set_pwm(.7,.7)

        #check forward
        lmr = [self.io.read(pin) for pin in pins]
        if sum(lmr) > 0:
            streets[0] = True
            
        #check left
        self.left_inplace()
        lmr = [self.io.read(pin) for pin in pins]
        if sum(lmr) > 0:
            streets[1] = True
            
        #check right
        self.backwards_inplace()
        lmr = [self.io.read(pin) for pin in pins]
        if sum(lmr) > 0:
            streets[3] = True
            
        #recenter
        self.left_inplace()
        
        return streets
        
    def known_map(self):
        #map = ['L', 'R', 'R', 'R', 'F']
        map = ['R', 'L', 'L', 'L', 'F']
        for dir in map:
            self.follow_tape()
            
            if dir == 'L':
                self.turn(1)
            elif dir == 'R':
                self.turn(-1)
            elif dir == 'F':
                self.turn(0)
        
        
    def follow_tape(self):
        '''sends eebot to go find some black tape and follow it'''
        prev = None
        
        SPIN = True
        SEARCH = True
        hit_line = False
        counter = 0
        spin_start_time = None
        while True:
            pins = self.LED_detectors
            lmr = [self.io.read(pin) for pin in pins]
            lr = [lmr[0], lmr[2]]

            if lmr != [0, 0, 0]:
                hit_line = True
                spin_start_time = None
                counter = 0
            
            #keep going straight
            if lmr == [0, 1, 0]:
                self.set(0.25, 0)
            #turn right
            elif lr == [0, 1]:
                prev = 'right'
                self.set(0.25, 90)
            #turn left
            elif lr == [1, 0]:
                prev = 'left'
                self.set(0.25, -90)
            
            #we hit an intersection
            elif lmr == [1, 1, 1]:
                self.set(0.25, 0)
                time.sleep(0.5)
                self.set(0,0)
                return 'INTERSECTION'
#                 if prev == 'left':
#                     self.set(0.25, -90)
#                 elif prev == 'right':
#                     self.set(0.25, 90)
            #we have reached a dead end
            elif lmr == [0, 0, 0] and hit_line:
                self.set(0, 0)
                #return 'DEAD'
#                 if not spin_start_time:
#                     spin_start_time = datetime.now()
#                 elif (datetime.now() - spin_start_time).seconds >= 5:
#                     hit_line = False
            #search
            elif lmr == [0, 0, 0] and not hit_line:
                spin_start_time = None
                self.set(.3, 30 - counter)
                counter += .0005

    def follow_directions(self, route):
        for dir in route:
            self.turn(dir)
            self.follow_tape()

    def scan(self, heading):
        # face backwards, since we know this will for sure have a road
        #self.turn(-2)

        samples = []
        start_time = datetime.now()
        time_delta = timedelta(0)

        #while time_delta.seconds + time_delta.microseconds * 1e-6 < 1.5:
        #    if int(time_delta.microseconds / 10000) % 3 == 0:
        #        samples.append(
        #            reduce(
        #                lambda led1, led2: led1 | led2,
        #                [ self.io.read(pin) for pin in self.LED_detectors ]
        #            )
        #        )
        #    time_delta = datetime.now() - start_time
        self.set_pwm(-.68,.68)
        for _ in range(100):
            sample = [ self.io.read(pin) for pin in self.LED_detectors ]
            print(sample)
            samples.append(
                reduce(
                    lambda led1, led2: led1 | led2,
                    sample
                )
            )
            time.sleep(0.0285)


        self.find_line(False)
        #self.set_pwm(0,0)
        print(len(samples))

        sample_div = round(len(samples)/15)
        streets = [
            1 in samples[:sample_div],
            1 in samples[3*sample_div:6*sample_div],
            1 in samples[7*sample_div:10*sample_div],
            1 in samples[11*sample_div:14*sample_div],
        ]
        streets = streets[-heading:] + streets[:-heading]

        print(streets)
        return streets
