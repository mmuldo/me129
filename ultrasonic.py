import time
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
    herd_directions : List[str]
        directions to turn
        'turn_right'
        'turn_left'
        'stay_straight'
    '''

    
    def __init__(self):
        self.state = ['ready', 'ready', 'ready']
        self.distance = [0, 0, 0]
        self.start_time = [0, 0, 0]
        self.cbrise = []
        self.cbfall = []
        self.io = pigpio.pi()
        self.herd_directions = ['straight', 'straight', 'straight']
        
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
            #print(self.distance[0], '0')
            self.state[0] = 'ready' 

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
            # print(self.distance[1], '1')
            self.state[1] = 'ready' 

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
            # print(self.distance[2], '2')
            self.state[2] = 'ready' 


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
        time.sleep(.1)


    def shutdown_ultrasonic(self):
        '''
        Cancels callback functions for each sensor.
        '''
        for i in range(3):
            self.cbrise[i].cancel()
            self.cbfall[i].cancel()

