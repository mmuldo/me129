import time
import pigpio

#ECHO_TRIGGER = [(36,33), (38,35), (40,37)] #This is not GPIO compatible
ECHO_TRIGGER = [(13,16), (20,19), (21,26)] #This is converted to GPIO compatible


class Ultrasonic:
    '''
    Represents an ultrasonic sensor
    
    Attributes
    ----------
    state : str
        'ready' -> ready to send trigger
        'await_rise' -> waiting for echo, rising edge
        'await_fall' -> waiting for echo, falling edge
    distance : int
        how far an object is detected from the robot
    start_time : int
        a counter, starting from when a trigger is sent
    '''
    
    def __init__(self):
        self.state = ['ready', 'ready', 'ready']
        self.distance = [0, 0, 0]
        self.start_time = [0, 0, 0]
        self.cbrise = []
        self.cbfall = []
        self.io = pigpio.pi()
        
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
        if self.state[0] == 'await_rise':
            #start the timer
            self.start_time[0] = tick
            self.state[0] = 'await_fall'

                
    def falling_0(self, gpio, level, tick):
        if self.state[0] == 'await_fall':
            end_time = tick
            delta_t = end_time - self.start_time[0]
            dist = 343/2 * delta_t * 1e-6
            self.distance[0] = dist
            print(self.distance[0], '0')
            self.state[0] = 'ready' 

#we can ignore level
    def rising_1(self, gpio, level, tick):
        if self.state[1] == 'await_rise':
            #start the timer
            self.start_time[1] = tick
            self.state[1] = 'await_fall'

                
    def falling_1(self, gpio, level, tick):
        if self.state[1] == 'await_fall':
            end_time = tick
            delta_t = end_time - self.start_time[1]
            dist = 343/2 * delta_t * 1e-6
            self.distance[1] = dist
            print(self.distance[1], '1')
            self.state[1] = 'ready' 

#we can ignore level
    def rising_2(self, gpio, level, tick):
        if self.state[2] == 'await_rise':
            #start the timer
            self.start_time[2] = tick
            self.state[2] = 'await_fall'

                
    def falling_2(self, gpio, level, tick):
        if self.state[2] == 'await_fall':
            end_time = tick
            delta_t = end_time - self.start_time[2]
            dist = 343/2 * delta_t * 1e-6
            self.distance[2] = dist
            print(self.distance[2], '2')
            self.state[2] = 'ready' 


    def trigger(self):
        '''
        sends a trigger and sets state to 'await_rise'
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
        for i in range(3):
            self.cbrise[i].cancel()
            self.cbfall[i].cancel()

