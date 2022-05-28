'''
controls triggering of ultrasonic sensors

Classes
-------
Ultrasonic: ultrasonic sensors
'''

# utility libraries
import time

# other libraries
import pigpio

#ECHO_TRIGGER = [(36,33), (38,35), (40,37)] #This is not GPIO compatible
ECHO_TRIGGER = [(13,16), (20,19), (21,26)] #This is converted to GPIO compatible


class Ultrasonic:
    '''
    Represents an ultrasonic sensor
    
    Attributes
    ----------
    state : List[str]
        'ready' -> ready to send trigger
        'await_rise' -> waiting for echo, rising edge
        'await_fall' -> waiting for echo, falling edge
    distance : List[int]
        list of distances recorded for each sensor
    start_time : List[int]
        list of time at which trigger is sent, for each sensor
	cbrise : List[]
	    list of rising callback functions
	cbfall : List[]
	    list of falling callback functions
    io : pigpio.io
        pi gpio instance

    Methods
    -------
    rising(i, gpio, level, tick): starts sensor i and awaits fall
    falling(i, gpio, level, tick): detects fall on sensor i
    trigger(): sends ultrasonic trigger
    shutdown_ultrasonic(): cancels callbacks for each ultrasonic sensor
    '''
    def __init__(self, passed_io: pigpio.pi):
        self.state = ['ready', 'ready', 'ready']
        self.distance = [0, 0, 0]
        self.start_time = [0, 0, 0]
        self.cbrise = []
        self.cbfall = []
        self.io = passed_io
        
        for echo, trig in ECHO_TRIGGER:
            # Set up the two pins as output/input.
            self.io.set_mode(trig, pigpio.OUTPUT)
            self.io.set_mode(echo, pigpio.INPUT)

        # Set up the interrupt handlers or callbacks.
        self.cbrise.extend([
            self.io.callback(
                ECHO_TRIGGER[i][0],
                pigpio.RISING_EDGE,
                lambda gpio, level, tick: self.rising(i, gpio, level, tick)
            )
            for i in range(len(ECHO_TRIGGER))
        ])

        self.cbrise.extend([
            self.io.callback(
                ECHO_TRIGGER[i][0],
                pigpio.FALLING_EDGE,
                lambda gpio, level, tick: self.falling(i, gpio, level, tick)
            )
            for i in range(len(ECHO_TRIGGER))
        ])

    def rising(self, i, gpio, level, tick):
        '''
        Records the start time for sensor i, which is when the echo is 
        pulled high. Sets the state to 'await_fall'.
        '''
        if self.state[i] == 'await_rise':
            #start the timer
            self.start_time[i] = tick
            self.state[i] = 'await_fall'

    def falling(self, i, gpio, level, tick):
        '''
	    Computes distance detected based on the time the falling
	    edge is sensed for sensor i. Sets state to 'ready'.
        '''
        if self.state[i] == 'await_fall':
            end_time = tick
            delta_t = end_time - self.start_time[i]
            dist = 343/2 * delta_t * 1e-6
            self.distance[0] = dist
            self.state[0] = 'ready' 

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

    def shutdown_ultrasonic(self):
        '''
        Cancels callback functions for each sensor.
        '''
        for i in range(len(ECHO_TRIGGER)):
            self.cbrise[i].cancel()
            self.cbfall[i].cancel()

