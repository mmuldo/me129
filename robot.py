'''
controls the behavior of our robot (eebot)

Classes
-------
EEBot: a robot capable of exploring and traversing maps using LED and
    ultrasonic sensors
'''

# project libraries
from util import *
import map
import ultrasonic

# utility libraries
from datetime import datetime, timedelta
from typing import List, Tuple
from functools import reduce
import math
import sys
import time

# other libraries
import pigpio


#################
### constants ###
#################
# ultrasonic consts
# TODO: check if these values are right (in meters)
INT_BLOCKED_DIST = .5
STR_BLOCKED_DIST = .2

# colors correspond to the color of the jumper cable connected to each LED
LED_PINS = {
    18: 'right',
    15: 'middle',
    14: 'left',
}

# motor consts
SPIN_PWM = 0.7
LIN_SPEED = 0.35
NEXT_INT_OVERSHOOT = 0.14 # seconds


###############
### classes ###
###############
class EEBot:
    '''eebot instance!

    Attributes
    ----------
    io : pigpio.pi
        pi I/O instance with which we can interact with our little eebot
    map : map.Map
        the map that the robot is traversing/exploring
    intersection : map.Intersection
        robot's current interseciton
    heading : int
        robot's current heading
    come_back : List[Intersections]
        queue of Intersections we need to explore more
    motors: List[int]
        GPIO pins that the motor leads are connected to;
        looking at the bot from the back, the pin connections in order
        are expected to be: [LEFT_NEG, LEFT_POS, RIGHT_NEG, RIGHT_POS]
    LED_detectors : List[int]
        GPIO pins that the LED/phototransistor detectors are connected to
    ultra : Ultrasonic
        ultrasonic detectors
    L : float
        distance from the rear axle to the caster wheel in meters
    d : float
        distance between the two wheels in meters

    Methods
    -------
    asses_blockage(moving): uses ultrasonics to determine a blockage
        directly ahead
    detectors_status(): gets reading of each LED detector
    set_pwm(leftdutycycle, rightdutycycle): sets PWM for both motors
    set(vel_nominal, steering_angle): sets vector-ed velocity
    adjust(): adjusts bot so that only middle sensor is over tape, if there
        is tape underneath
    next_intersection(): drives from one intersection to the interseciton
        directly ahead
    spin_kick(spin_right): maxes out pwm to get motors going to help with 
        spinning
    snap(spin_right): starts over black tape and spins to next black tape,
        reporting the amount of degrees spun
    turn(direction): turns the bot in the direction indicated
    first_scan(): very first scan when first building a map (doesn't assume
        we have black tape directly behind)
    partial_scan(): intersection scan that assumes we have black tape 
        directly behind
    follow_directions(route): tells bot to follow a sequence of directions
        on a map
    goto(dest): tells bot to go to destination intersection (if possible)
    find(dest): tells bot to go to destination in unexplored map
    shutdown(): stops robot
    '''
    def __init__(
        self,
        passed_io: pigpio.pi,
        botMap: map.Map = None,
        intersection: map.Intersection = map.Intersection((0,0)),
        heading: int = S,
        come_back: List[map.Intersection] = None,
        L: float = 0.103,
        d: float = 0.131,
    ):
        '''initializes eebot

        Parameters
        ----------
        passed_io : pigpio.pi
            pi gpio instance
        botMap : map.Map, optional
            the map that the robot is traversing/exploring
            default is map containing only current intersection
        intersection : map.Intersection, optional
            robot's current interseciton
            default is (0,0), unblocked, with unknown streets
        heading : int, optional
            robot's current heading
            default is S
        L : float, optional
            length in m from wheels to ball bearing
            default is what we measured in lab
        d : float, optional
            length in m between wheels
            default is what we measured in lab
        '''
        self.motors = [pin for motor in MOTOR_PINS for pin in motor]
        self.LED_detectors = list(LED_PINS.keys())
        self.map = botMap
        self.intersection = intersection
        self.heading = heading
        self.come_back = come_back
        self.L = L
        self.d = d
        self.io = passed_io
        self.ultra = ultrasonic.Ultrasonic(passed_io)

        # map init
        if not botMap:
            self.map = map.Map([self.intersection])
        if not come_back:
            self.come_back = [self.intersection]

        # make sure gpio is connected
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)

        # set up motor pins
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

        # set up leds
        for pin in self.LED_detectors:
            # setup four LED pins as input
            self.io.set_mode(pin, pigpio.INPUT)


    ##############
    ## blockage ##
    ##############
    def assess_blockage(self, moving: bool = False) -> bool:
        '''
        uses ultrasound to scan for forward blockage, and updates map 
        accordingly

        Parameters
        ----------
        moving : bool
            True -> bot is currently line following
            False -> bot is currently stationary

        Returns
        -------
        bool
            True --> map same as before
            False --> map changed upon assessment
        '''
        # save original map to check if it's changed at the end
        m = self.map.copy()

        # return if the forward neighbor is neither registered as 
        # present or blocked
        print(self.intersection, 'intersection')
        if not self.intersection.streets[self.heading] > ABSENT:
            return True

        # get intersection that is directly ahead
        forward_neighbor = self.map.neighbors(
            self.intersection
        )[self.heading]
        


        # read ultrasound
        self.ultra.trigger()
        dist_to_obstacle = self.ultra.distance[1]

        # if moving and we see an obstacle, the best we can do is assume 
        # intersection is blocked (and not the street)
        if moving:
            # here, we use the blocking distance indicative of a street
            # blockage because we could be like halfway through the
            # intersection or something when the obstacle pops up
            forward_neighbor.blocked = dist_to_obstacle < STR_BLOCKED_DIST
            # and don't do any of the other fancy stuff that distinguishes
            # an intersection blockage from a street blockage
            return m == self.map

        # if stationary, we can more accurately distinguish between
        # a blocked intersection vs a blocked street.
        # the first step is to try and detect an intersection blockage
        print(forward_neighbor, 'neighbor')
        forward_neighbor.blocked = dist_to_obstacle < INT_BLOCKED_DIST

        # next, try and detect a street blockage, and update the street info
        # for both intersections accordingly
        if dist_to_obstacle < STR_BLOCKED_DIST:
            self.intersection.streets[self.heading] = BLOCKED
            forward_neighbor.streets[(-self.heading)%4] = BLOCKED
        else:
            self.intersection.streets[self.heading] = PRESENT
            forward_neighbor.streets[(-self.heading)%4] = PRESENT

        # if the map changed, return false to indicate we gained new info
        return m == self.map


    ###################
    ## LED detecting ##
    ###################
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



    ###################
    ## motor control ##
    ###################
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
        for motor_number, dutycycle in enumerate([
            leftdutycycle, 
            rightdutycycle
        ]):
            # convert a (motor_number, dutycycle) pair to a
            # ((motor_number, pwm), (motor_number, pwm)) thing
            # i.e. this contains the pwm info for both leads of both
            # motors
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


    ####################
    ## linear driving ##
    ####################
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

    def next_intersection(self, visualize: bool = False) -> bool:
        '''
        sends eebot to next intersection

        Parameters
        ----------
        visualize : bool, optional
            if true, will update the matplotlib visualization of the map

        Preconditions
        -------------
        assumes we are currently on a street (there's black tape under
        the LEDs)

        Returns
        -------
        bool
            True --> reached next intersection without having to abort
            False --> have to abort current process because we have new info
        '''
        while True:
            # read LED detectors
            left, middle, right = self.detectors_status()
            # read the ultrasonic sensors
            self.ultra.trigger()
            
            # check for obstacle directly ahead
            if self.ultra.distance[1] < STR_BLOCKED_DIST:
                # this is called to update the map
                self.assess_blockage(moving=True)
                if visualize:
                    self.map.visualize()
                    print('VISUALIZE')

                # turn around
                self.turn(B)
                self.heading = (self.heading + B)%4

                # go back to previous intersection
                self.next_intersection()

                # abort process
                return False

            # normal line following
            if not left and middle and not right:
                # keep going straight
                self.set(LIN_SPEED, 0)
                #count = 0
            elif not left and right:
                # veer right
                self.set(0.5, -10)
                #count = 0
            elif left and not right:
                # veer left
                self.set(0.5, 10)
                #count = 0
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

                # update new intersection
                self.intersection = self.map.get_neighbor(
                    self.intersection, 
                    self.heading
                )
                # current intersection should be valid and not None
                #assert self.intersection is not None 
                #ignore for now??? DYLAN

                # successfully reached next intersection
                return True

            else:
                #TODO: use tunnel following here
                self.set_pwm(0,0)


    #############
    ## turning ##
    #############
    def spin_kick(self, spin_right: bool):
        '''maxes out pwm spin for a short period to get motors going

        Parameters
        -------
        spin_right : bool
            True --> spin right
            False --> spin left
        '''
        # sign is negative for right spin, positive for left spin
        sign = -1 if spin_right else 1
        self.set_pwm(-sign, sign)
        time.sleep(0.1)
        # assumes caller sets pwm immediately after calling this function

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
        # sign is negative for right spin, positive for left spin
        spin_sign = -1 if spin_right else 1
        # give motors a kick
        self.spin_kick(spin_right)
        # then do actual controlled spin in place
        self.set_pwm(- spin_sign * pwm, spin_sign * pwm)
        # initialize the outside led off flag to True (by assumption) (this
        #   is for the timers we use later)
        outside_LED_off = True
        # initialize the following just to avoid python warnings
        t1 = t2 = ratio = degrees = 0

        # start global timer
        start = time.perf_counter()
        while True:
            # read each LED detector
            left, middle, right = self.detectors_status()

            # wait for outside LED to hit black tape
            if (
                (not spin_right and left) or 
                (spin_right and right)
            ) and outside_LED_off:
                # clock t1
                t1 = time.perf_counter() - start
                outside_LED_off = False

            # wait for middle sensor to reach center of the black tape
            #   we're spinning towards
            if not left and middle and not right and not outside_LED_off:
                # clock t2
                t2 = time.perf_counter() - start
                ratio = t2/(t2-t1)
                break

            # fail safe to break out of loop in case something goes wrong
            if time.perf_counter() - start > 20:
                # shouldn't ever hit this, but just in case
                break

        # stop motors
        self.set_pwm(0,0)

        # in theory, the width of the tape corresponds to around 20 degrees,
        # so the the ratio of a spin in right angle increments to the spin
        # of a single LED across the tape is 90/20 = 4.5, 180/20 = 9,
        # 270/20 = 13.5, and 360/20 = 18
        print(ratio)
        if ratio < 5.9:
            # should be around 4.5 in this case
            degrees = 90
        elif 5.9 <= ratio < 10.5:
            # should be around 9 in this case
            degrees = 180
        elif 10.5 <= ratio < 14.1:
            # should be around 13.5 in this case
            degrees = 270
        else:
            degrees = 360
        return degrees

    def turn(self, direction: int, visualize: bool = False) -> bool:
        '''
        turn in the specified direction

        Parameters
        ----------
        value : int
            0, --> forwards
            1, --> left
            2, --> backwards
            3, --> right
        visualize : bool, optional
            set to true to update and view map visualization during turn

        Returns
        -------
        bool
            True --> completed turn without having to abort
            False --> have to abort current process because we have new info;
                it's possible to have made a turn, but the turn to be
                considered "aborted" if new info was collected on the map
                during the process
        '''
        direction = direction % 4

        # assess blockage prior to turning
        if not self.assess_blockage():
            if visualize:
                self.map.visualize()
            return False

        if direction == R:
            # if attempting to turn right, snap should return 90 degrees
            assert self.snap(True, SPIN_PWM) == 90
        elif direction == L:
            # if attempting to turn left, snap should return 90 degrees
            assert self.snap(False, SPIN_PWM) == 90
        elif direction == B:
            # if attempting to turn around, get the amount turned:
            #   could be 90 after first snap if there was black tape in path
            #   will be 180 if no black tape in between path
            # the choice to spin right is arbitrary
            degree = self.snap(True, SPIN_PWM)

            if degree == 90:
                # scan while we're here i guess
                self.heading = (self.heading + R)%4
                if not self.assess_blockage():
                    if visualize:
                        self.map.visualize()
                    # abort if new info collected on map
                    return False

                # if there was black tape in bath, turn again and ensure
                #   snap returns 90 degrees
                assert self.snap(True, SPIN_PWM) == 90

                # scan and update again
                self.heading = (self.heading + R)%4
                if not self.assess_blockage():
                    if visualize:
                        self.map.visualize()
                    # abort if new info collected on map
                    return False

                # if didn't have to abort, return here since there's lots
                #   of edge case logic
                return True
            else:
                # if attempting to around and there wasn't any black tape
                # in between, snap should return 180 degrees
                assert degree == 180

        # update heading
        self.heading = (self.heading + direction)%4
        # assess blockage after turning
        if not self.assess_blockage():
            if visualize:
                self.map.visualize()
            # abort if new info collected on map
            return False

        # if didn't have to abort during turn, report such
        return True


    ##############
    ## scanning ##
    ##############
    def first_scan(self) -> List[int]:
        '''
        specifically for the first scan of a map building;
        in particular, doesn't require black tape to be 180 degrees away;
        check for adjacent streets at an intersection;
        also updates map if obstacles are found (but doesn't abort)

        Returns
        -------
        List[int]
            length 4 list where each element indicates
            if there is a street at [North, West, South, East].
            -1 --> Unknown
            0 --> no street
            1 --> street exists
        '''
        # initialize street info
        streets = [UNKNOWN] * 4

        # check forward direction
        streets[self.heading] = PRESENT if any(
            self.detectors_status()
        ) else ABSENT

        # scan for forward obstacles
        self.assess_blockage()

        # set directions based on snap amount
        turn_amount = self.snap(spin_right=True, pwm=SPIN_PWM)

        if turn_amount == 90:
            # turned right
            # update heading
            self.heading = (self.heading + R)%4
            streets[self.heading] = PRESENT
            # scan for right obstacles
            self.assess_blockage()
        elif turn_amount == 180:
            # turned backwards
            # update heading
            self.heading = (self.heading + B)%4
            streets[(self.heading + 1)%4] = ABSENT
            streets[self.heading] = PRESENT
            # scan for back obstacles
            self.assess_blockage()
        elif turn_amount == 270:
            # turned left
            # update heading
            self.heading = (self.heading + L)%4
            streets[(self.heading + 2)%4] = ABSENT
            streets[(self.heading + 1)%4] = ABSENT
            streets[self.heading] = PRESENT
            # scan for left obstacles
            self.assess_blockage()
        else:
            # facing forward
            # update heading
            self.heading = (self.heading + L)%4
            streets[(self.heading + 3)%4] = ABSENT
            streets[(self.heading + 2)%4] = ABSENT
            streets[(self.heading + 1)%4] = ABSENT
            # scan for forward obstacles again just for kicks
            self.assess_blockage()

        return streets

    def partial_scan(
        self,
        check_right: bool,
    ) -> List[int]:
        '''
        check for adjacent streets at an intersection;
        also scans for obstacles (doesn't abort however)

        Parameters
        ----------
        spin_right : bool
            True --> check right street
            False --> check left street

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
        # initial heading
        initial_heading = self.heading

        # wait a beat to make it clear we're scanning
        time.sleep(0.5)

        streets = [UNKNOWN] * 4
        # assume there is black tape directly behind (since we just came
        # from there)
        streets[B] = PRESENT

        # check forward direction
        streets[F] = PRESENT if any(self.detectors_status()) else ABSENT
        # scan for forward obstacles
        self.assess_blockage()

        # check 90 degree direction
        direction = R if check_right else L
        turn_amount = self.snap(spin_right=check_right, pwm=SPIN_PWM)
        # update heading:
        #   if we didn't turn 90, we turned 180, so facing backwards
        self.heading = (self.heading + direction)%4 if (
            turn_amount == 90
        ) else (self.heading + B)%4
        # scan for obstacles
        self.assess_blockage()
        # update street info
        streets[direction] = PRESENT if turn_amount == 90 else ABSENT

        # realign streets based on initial heading (so that they're 
        #   [N, W, S, E])
        streets = streets[-initial_heading:] + streets[:-initial_heading]

        # wait a beat to make it clear we're scanning
        time.sleep(0.5)
        return streets


    #########################
    ## driving around town ##
    #########################
    def follow_directions(self, route: List[int], visualize: bool = False):
        '''
        follow a sequence of turns like so:
            turn1 --> drive straight --> turn2 --> drive straight --> ...

        Parameters
        ----------
        route : List[int]
            the sequence of turns (forward, left, backward, right) to follow
        visualize : bool, optional
            set to true to update/view map vizualization while following
            directions

        Returns
        -------
        bool
            True --> completed directions successfully
            False --> have to abort current process because we have new info
        '''
        for dir in route:
            if not self.turn(dir, visualize):
                # abort if turn was aborted
                return False
            if not self.next_intersection(visualize):
                # abort if line following was aborted
                return False
        # success!
        return True

    def goto(
        self,
        dest: map.Intersection,
        visualize: bool = False
    ) -> bool:
        '''
        sends eebot from current location to dest, if possible
    
        Parameters
        ----------
        start : Intersection
            (long, lat) starting point
        dest : Intersection
            (long, lat) ending point
        visualize : bool, optional
            set to true to update/view map vizualization while traveling
    
        Returns
        -------
        bool
            True --> successfully went from current intersection to dest
            False --> not possible
        '''
        # calculate shortest route (as far as we know)
        route = self.map.shortest_route(self.intersection, dest)
        if not route:
            # if no way to get there, abort and await further instructions
            return False

        # convert the route to directions
        directions, _ = map.route_to_directions(route, self.heading)
        # attempt to follow directions (may run into obstacle)
        while not self.follow_directions(directions, visualize):
            # keep trying until we get there
            route = self.map.shortest_route(self.intersection, dest)
            if not route:
                # if no way to get there, abort and await further 
                #   instructions
                return False
            directions, _ = map.route_to_directions(route, self.heading)

        # success!
        return True

    def find(
        self,
        dest: Tuple[int, int],
        visualize: bool = False
    ):
        '''
        attempts to go from current intersection to dest in map that
        hasn't necessarily been fully explored
    
        Parameters
        ----------
        dest : Tuple[int, int]
            (long, lat) ending point
        visualize : bool, optional
            set to true to update/view map vizualization while traveling

        Preconditions
        -------------
        assumes there is black tape behind bot to start (so we can snap to 
            tape that is 180 degrees)
        '''
        # check if we already know everything about the dest intersection
        dest_int = self.map.get_intersection(dest)
        if dest_int and UNKNOWN not in dest_int.streets:
            # if so, just goto it
            return self.goto(dest_int, visualize)

        # helper function
        def update_neighbors(
            accessibility: int, 
            neighbors: List[Tuple[int,int]]
        ):
            '''
            updates neighbors of the current intersection after scan;
            helper function for find

            Parameters
            ----------
            accessibility : int
                PRESENT or BLOCKED
            neighbors: List[Tuple[int,int]]
                list of neighbors coords: either accessible list or blocked 
                list
            '''
            # update neighbors as well
            for neighbor_coords in neighbors:
                # get neighbor from coords in map (if it's in there)
                neighbor = self.map.get_intersection(neighbor_coords)
    
                if neighbor:
                    # this neighbor is already in map, so update streets
                    neighbor.streets[
                        neighbor.direction(self.intersection)
                    ] = accessibility
    
                    # remove neighbor from come_back queue if necessary
                    if (
                        UNKNOWN not in neighbor.streets and 
                        neighbor in self.come_back
                    ):
                        self.come_back.remove(neighbor)
                else:
                    # neighbor not yet in map, so initialize it
                    neighbor = map.Intersection(neighbor_coords)
                    neighbor.streets[
                        neighbor.direction(self.intersection)
                    ] = accessibility
                    self.map.intersections.append(neighbor)
    
                    # we'll need to come back to this later
                    self.come_back.append(neighbor)

        while self.intersection != dest or self.come_back:
            # next intersection to explore is at the top of the 
            # come_back queue
            next = self.come_back[0]
            while not self.goto(next, visualize):
                # if we failed to get to next...
                # rotate queue (putting the thing we were trying to goto
                #   at the end)
                self.come_back = self.come_back[1:] + self.come_back[:1]
                next = self.come_back[0]

            # if the street to the right is unknown, check right in scan
            # otherwise, check left in scan
            check_right = self.intersection.streets[
                (self.heading + R)%4
            ] == UNKNOWN
            street_info = self.partial_scan(check_right)
    
            # update streets with new info
            for d in range(len(self.intersection.streets)):
                if street_info[d] != UNKNOWN:
                    self.intersection.streets[d] = street_info[d]
    
            # update neighbors and blocked neighbors as well
            update_neighbors(PRESENT, self.intersection.neighbors())
            update_neighbors(BLOCKED, self.intersection.blocked_neighbors())

            # update map visualization
            if visualize:
                self.map.visualize()
    
            self.come_back.remove(self.intersection)

            # sort come_back queue based on closest to dest
            self.come_back.sort(key = lambda i: i.distance(dest))

            # check if we need to come back to current at some point
            if UNKNOWN in self.intersection.streets:
                self.come_back.append(self.intersection)



    ############################
    ## shutdown everything :( ##
    ############################
    def shutdown(self):
        '''stops robot'''
        print("Turning off...")

        for pin in self.motors:
            self.io.set_PWM_dutycycle(pin, 0)

        self.io.stop()
