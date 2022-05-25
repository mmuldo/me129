import pigpio
import sys
import math
import time
from datetime import datetime, timedelta
from typing import List, Tuple
from functools import reduce
ECHO_TRIGGER = [(13,16), (20,19), (21,26)]

# first element is the left motor; second is the right.
# first element of each tuple is the negative terminal; second is positive
MOTOR_PINS = [(7, 8), (5, 6)]

# colors correspond to the color of the jumper cable connected to each LED
LED_PINS = {
    18: 'right',
    15: 'middle',
    14: 'left',
}

# util consts
SPIN_PWM = 0.7
LIN_SPEED = 0.35
NEXT_INT_OVERSHOOT = 0.14/.25 # seconds

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

# existence of streets
UNKNOWN = -1
ABSENT = 0
PRESENT = 1

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
    heading : int
        current heading of the robot
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
        passed_io: pigpio.pi,
        L: float = 0.103,
        d: float = 0.131,
        
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
        self.heading = N # initialize the current heading to North.
        self.io = passed_io
        self.state = ['ready', 'ready', 'ready']
        self.distance = [0, 0, 0]
        self.start_time = [0, 0, 0]
        self.cbrise = []
        self.cbfall = []

        for echo, trig in ECHO_TRIGGER:
            # Set up the two pins as output/input.
            self.io.set_mode(trig, pigpio.OUTPUT)
            self.io.set_mode(echo, pigpio.INPUT)
        # Set up the interrupt handlers or callbacks.
        self.cbrise.append(self.io.callback(ECHO_TRIGGER[0][0], pigpio.RISING_EDGE, self.rising_0))
        self.cbfall.append(self.io.callback(ECHO_TRIGGER[0][0], pigpio.FALLING_EDGE, self.falling_0))
        
        self.cbrise.append(self.io.callback(ECHO_TRIGGER[1][0], pigpio.RISING_EDGE, self.rising_1))
        self.cbfall.append(self.io.callback(ECHO_TRIGGER[1][0], pigpio.FALLING_EDGE, self.falling_1))
        
        self.cbrise.append(self.io.callback(ECHO_TRIGGER[2][0], pigpio.RISING_EDGE, self.rising_2))
        self.cbfall.append(self.io.callback(ECHO_TRIGGER[2][0], pigpio.FALLING_EDGE, self.falling_2))


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

    #we can ignore level
    def rising_0(self, gpio, level, tick):
        '''
        Records the start time for sensor 0, which is when the echo is pulled high.
        Sets the state to 'await_fall'.
        '''
        if self.state[0] == 'await_rise':
            #start the timer
            self.start_time[0] = tick
            self.state[0] = 'await_fall'
        # else:
        #     raise Exception('illegal rise 0')

                
    def falling_0(self, gpio, level, tick):
        '''
	    Computes distance detected based on the time the falling
	    edge is sensed for sensor 0. Sets state to 'ready'.
        '''
        if self.state[0] == 'await_fall':
            end_time = tick
            delta_t = end_time - self.start_time[0]
            dist = 343/2 * delta_t * 1e-6
            self.distance[0] = dist
            self.state[0] = 'ready' 
        # else:
        #     raise Exception('illegal fall 0')

#we can ignore level
    def rising_1(self, gpio, level, tick):
        '''
        Records the start time for sensor 1, which is when the echo is pulled high.
        Sets the state to 'await_fall'.
        '''
        if self.state[1] == 'await_rise':
            #start the timer
            self.start_time[1] = tick
            self.state[1] = 'await_fall'
        # else:
        #     raise Exception('illegal rise 1')

                
    def falling_1(self, gpio, level, tick):
        '''
	    Computes distance detected based on the time the falling
	    edge is sensed for sensor 1. Sets state to 'ready'.
        '''
        if self.state[1] == 'await_fall':
            end_time = tick
            delta_t = end_time - self.start_time[1]
            dist = 343/2 * delta_t * 1e-6
            self.distance[1] = dist
            self.state[1] = 'ready' 
        # else:
        #     raise Exception('illegal fall 1')

#we can ignore level
    def rising_2(self, gpio, level, tick):
        '''
        Records the start time for sensor 2, which is when the echo is pulled high.
        Sets the state to 'await_fall'.
        '''
        if self.state[2] == 'await_rise':
            #start the timer
            self.start_time[2] = tick
            self.state[2] = 'await_fall'
        # else:
        #     raise Exception('illegal rise 2')

                
    def falling_2(self, gpio, level, tick):
        '''
	    Computes distance detected based on the time the falling
	    edge is sensed for sensor 2. Sets state to 'ready'.
        '''
        if self.state[2] == 'await_fall':
            end_time = tick
            delta_t = end_time - self.start_time[2]
            dist = 343/2 * delta_t * 1e-6
            self.distance[2] = dist
            self.state[2] = 'ready' 
        # else:

        #     raise Exception('illegal fall 2')


    def trigger(self):
        '''
        Sends a trigger and sets state to 'await_rise'
        '''
        counter = -1
        for echo, trig in ECHO_TRIGGER:
            counter += 1
            if self.state[counter] == 'ready':
                # Pull one (or all) trigger pins HIGH
                self.io.write(trig, 1)
                # Hold for 10microseconds.
                time.sleep(0.000010)
                # Pull the pins LOW again.
                self.io.write(trig, 0)
                # Update state to await rising
                self.state[counter] = 'await_rise'
            #else:
            #    raise Exception(f'illegal trigger {counter}' + str(self.state))
        #  time.sleep(.1)


    def shutdown_ultrasonic(self):
        '''
        Cancels callback functions for each sensor.
        '''
        for i in range(3):
            self.cbrise[i].cancel()
            self.cbfall[i].cancel()

    def shutdown(self):
        '''stops robot'''
        print("Turning off...")

        for pin in self.motors:
            self.io.set_PWM_dutycycle(pin, 0)

        self.io.stop()


    def detectors_status(self) -> Tuple[bool, bool, bool]:
        '''gets reading of each detector

        Returns
        -------
        Tuple[bool, bool, bool]
            (left, middle, right) LED reading.
            True --> LED on (over black tape)
            False --> LED off (over white floor)
        '''
        return tuple(
            bool(self.io.read(pin)) 
            for pin in sorted(self.LED_detectors)
        )

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

        k = 1.5
        fric = .34 if vel_nominal > 0 else -.34

        if vel_nominal == 0:
            self.set_pwm(0,0)
        else: 
            self.set_pwm(
                k*vel_left + fric,
                k*vel_right + fric
            )

    def spin_kick(self, spin_right: bool):
        '''maxes out pwm spin for a short period to get motors going

        Parameters
        -------
        spin_right : bool
            True --> spin right
            False --> spin left
        '''
        sign = -1 if spin_right else 1
        self.set_pwm(-sign * 1, sign*1)
        time.sleep(0.1)




    def turn(self, direction: int):
        '''
        turn in the specified direction

        Parameters
        ----------
        value : int
            0, --> forwards
            1, --> left
            2, --> backwards
            3, --> right
        '''
        direction = direction % 4
        if direction == R:
            assert self.snap(True, SPIN_PWM) == 90
        elif direction == L:
            assert self.snap(False, SPIN_PWM) == 90
        elif direction == B:
            degree = self.snap(True, SPIN_PWM)
            if degree == 90:
                assert self.snap(True, SPIN_PWM) == 90
            else:
                assert degree == 180


    def find_line(self, spin_right: bool, wait_time: float) -> bool:
        '''
        spin until a line is found

        Parameters
        ----------
        spin_right : bool
            True --> spin right
            False --> spin left
        wait_time : float
            amount of seconds to spin for before giving up

        TODO
        ----
        check PWM vals
        '''
        sign = -1 if spin_right else 1
        self.set_pwm(-sign*0.7, sign*0.7)

        start_time = datetime.now()
        rml = [0, 0, 0]
        while rml[1] != 1:
            rml = [self.io.read(pin) for pin in self.LED_detectors]
            time_delta = datetime.now() - start_time
            if time_delta.seconds + time_delta.microseconds * 1e-6 >= wait_time:
                self.set_pwm(0, 0)
                return False
        self.set_pwm(0, 0)
        return True

    def get_off_line(self, spin_right: bool):
        '''
        if currently over a line, spin until off of it

        Parameters
        ----------
        spin_right : bool
            True --> spin right
            False --> spin left
        '''
        sign = -1 if spin_right else 1
        self.set_pwm(-sign*0.7, sign*0.7)
        while 1 in [self.io.read(pin) for pin in self.LED_detectors]:
            pass
        self.set_pwm(0,0)

    def snap(self, spin_right: bool, pwm: float) -> int:
        '''
        starts spinning and snaps to nearest black tape.
        if currently over black tape, spins off and then starts
        searching for black tape.

        Parameters
        ----------
        spin_right : bool
            True --> spin right
            False --> spin left
        pwm : float
            pwm to spin at (between 0.0 and 1.0)

        Returns
        -------
        int
            degree amount turned, in 90 degree increments
            i.e. returns 90, 180, 270, or 360

        TODO
        ----
        still need to check cutoffs
        '''
        spin_sign = -1 if spin_right else 1
        self.spin_kick(spin_right)
        self.set_pwm(- spin_sign * pwm, spin_sign * pwm)
        outside_LED_off = True

        start = time.perf_counter()
        while True:
            left, middle, right = self.detectors_status()
            if (
                (not spin_right and left) or (spin_right and right)
            ) and outside_LED_off:
                t1 = time.perf_counter() - start
                outside_LED_off = False
            if not left and middle and not right and not outside_LED_off:
                t2 = time.perf_counter() - start
                ratio = t2/(t2-t1)
                break
            if time.perf_counter() - start > 20:
                # shouldn't ever hit this, but just in case
                break
        # print(f't1: {t1}')
        # print(f't2: {t2}')
        # print(f't2-t1: {t2-t1}')
        self.set_pwm(0,0)

        # in theory, the width of the tape corresponds to around 20 degrees,
        # so the the ratio of a spin in right angle increments to the spin
        # of a single LED across the tape is 90/20 = 4.5, 180/20 = 9,
        # 270/20 = 13.5, and 360/20 = 18
        print(ratio)
        if ratio < 5.9:
            # should be around 4.5 in this case
            self.heading = (self.heading + spin_sign*1) % 4
            return 90
        elif 5.9 <= ratio < 10.5:
            # should be around 9 in this case
            self.heading = (self.heading + spin_sign*2) % 4
            return 180
        elif 10.5 <= ratio < 14.1:
            # should be around 13.5 in this case
            self.heading = (self.heading + spin_sign*3) % 4
            return 270
        else:
            return 360

        # shouldn't reach this point
        assert False


    def adjust(self):
        '''
        if over tape, adjusts bot such that only middle sensor is over
        if not over tape, does nothing
        '''
        left, middle, right = self.detectors_status()
        if not left and not middle and not right:
            return

        while self.detectors_status()[0]:
            # adjust left
            self.set_pwm(-0.65, 0.65)

        while self.detectors_status()[2]:
            # adjust right
            self.set_pwm(0.65, -0.65)

        self.set_pwm(0,0)

    def next_intersection(self, change_route) -> bool:
        '''
        sends eebot to next intersection

        Preconditions
        -------------
        assumes we are currently on a street (there's black tape under
        the LEDs)

        Returns
        -------
        bool
            True --> successfully reached an intersection
            False --> got off the map somehow (fail)
        '''
        count = 0
        print('in here')
        while True:
            left, middle, right = self.detectors_status()
            # if change_route[0] == True:
            #     break

            #TODO READ THE ULTRA SONIC SENSORS HERE
            self.trigger()
            

            #check if distance
            if self.distance[1] < .2:
                self.set(0,0)
            #continue with the normal functions    
            if not left and middle and not right:
                # keep going straight
                self.set(LIN_SPEED, 0)
                count = 0
            elif not left and right:
                # veer right
                self.set(0.25, -90)
                count = 0
            elif left and not right:
                # veer left
                self.set(0.25, 90)
                count = 0
            elif left and middle and right:
                # reached intersection
                # drive forward a little so wheels are over intersection
                self.set(0.25, 0)
                # need to travel ~0.153 meters
                time.sleep(NEXT_INT_OVERSHOOT)

                # stop
                self.set_pwm(0,0)

                # adjust such that only middle sensor over tape (if possible)
                self.adjust()

                return True
            elif not left and not middle and not right:
                # off map
                # stop
                if count < 50:
                    self.set(LIN_SPEED, 0)
                    count += 1
                else:
                    print('off the map')
                    self.set_pwm(0,0)
                    return False
            else:
                # shouldn't hit this because we exhausted all valid cases
                # only non valid case is
                #   left and not middle and right
                assert False


    def follow_tape(self):
        '''sends eebot to go find some black tape and follow it'''
        prev = None
        
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

    def follow_directions(self, route, change_route):
        '''
        follow a sequence of turns like so:
            turn1 --> drive straight --> turn2 --> drive straight --> ...

        Parameters
        ----------
        route : List[int]
            the sequence of turns (forward, left, backward, right) to follow
        '''
        prev_route = []
        for dir in route:
            prev_route.append(dir)
            # print('continue')
            # print(change_route[0])
            # if change_route[0] == True:
                
            #     change_route[0] = False
            #     print('we change')
            #     prev_route.reverse()
            #     for new_dir in prev_route:
            #         self.turn(new_dir)
            #         self.next_intersection(change_route)
            #     break
            self.turn(dir)
            self.next_intersection(change_route)

    def partial_scan(
        self,
        check_right: bool,
        heading: int
    ) -> Tuple[List[int], int]:
        '''
        check for adjacent streets at an intersection

        Parameters
        ----------
        spin_right : bool
            True --> check right street
            False --> check left street
        heading : int
            initial heading prior to scanning

        Preconditions
        -------------
        Assumes at an intersection and there is black tape 180 degrees away

        Returns
        -------
        Tuple[List[int], int]
            first item is a length 4 list where each element indicates
            if there is a street at [North, West, South, East].
            -1 --> Unknown
            0 --> no street
            1 --> street exists

            second item is the new heading after scanning
        '''
        # update robot's current heading
        self.heading = heading

        # wait a beat to make it clear we're scanning
        time.sleep(0.5)

        streets = [UNKNOWN] * 4
        # assume there is black tape directly behind (since we just came
        # from there)
        streets[B] = PRESENT

        # check forward direction
        streets[F] = PRESENT if any(self.detectors_status()) else ABSENT

        # check 90 degree direction
        direction = R if check_right else L
        turn_amount = self.snap(spin_right=check_right, pwm=SPIN_PWM)
        streets[direction] = PRESENT if turn_amount == 90 else ABSENT

        # realign streets based on heading (so that they're [N, W, S, E]
        streets = streets[-heading:] + streets[:-heading]

        # readjust heading
        if turn_amount == 90:
            new_heading = (heading + direction) % 4
        elif turn_amount == 180:
            new_heading = (heading + B) % 4
        else:
            # turn_amount should be 90 or 180
            assert False

        # wait a beat to make it clear we're scanning
        time.sleep(0.5)
        self.heading = new_heading
        return (streets, new_heading)

    def first_scan(self):
        '''
        specifically the first scan of a map building
        in particular, doesn't require black tape to be 180 degrees away
        check for adjacent streets at an intersection

        Preconditions
        -------------
        Assumes facing north

        Returns
        -------
        Tuple[List[int], int]
            first item is a length 4 list where each element indicates
            if there is a street at [North, West, South, East].
            -1 --> Unknown
            0 --> no street
            1 --> street exists

            second item is the new heading after scanning
        '''
        # update robot's heading
        self.heading = N

        streets = [UNKNOWN] * 4

        # check forward direction
        streets[N] = PRESENT if any(self.detectors_status()) else ABSENT

        # set directions based on snap amount
        turn_amount = self.snap(spin_right=True, pwm=SPIN_PWM)
        if turn_amount == 90:
            streets[E] = PRESENT
            heading = E
        elif turn_amount == 180:
            streets[E] = ABSENT
            streets[S] = PRESENT
            heading = S
        elif turn_amount == 270:
            streets[E] = ABSENT
            streets[S] = ABSENT
            streets[W] = PRESENT
            heading = W
        else:
            streets[E] = ABSENT
            streets[S] = ABSENT
            streets[W] = ABSENT
            heading = N

        self.heading = heading
        return (streets, heading)
