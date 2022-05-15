import time
import pigpio

#ECHO_TRIGGER = [(36,33), (38,35), (40,37)] #This is not GPIO compatible
ECHO_TRIGGER = [(13,16), (20,19), (21,26)] #This is converted to GPIO compatible

cbrise = []
cbfall = []
io = pigpio.pi()
start_time = {}
end_time = {}

def distance():
    for echo, trig in ECHO_TRIGGER:
        print(echo, trig)
        # Set up the two pins as output/input.
        io.set_mode(trig, pigpio.OUTPUT)
        io.set_mode(echo, pigpio.INPUT)
        
        #io.set_pull_up_down(echo, pigpio.PUD_DOWN)

        # # Set up the interrupt handlers or callbacks.
        cbrise.append(io.callback(echo, pigpio.RISING_EDGE, rising))
        cbfall.append(io.callback(echo, pigpio.FALLING_EDGE, falling))
       
        io.write(trig,1)
        time.sleep(.00001)
        io.write(trig,0)
        

        
#we can ignore level
def rising(gpio, level, tick):
    print('rise')

    #start the timer
    start_time[gpio] = tick
    
def falling(gpio, level, tick):
    print('fall')
    end_time[gpio] = tick
    delta_t = end_time[gpio] - start_time[gpio]
    dist = 343/2 * delta_t * 1e-6
    
    if dist < .5:
        pass
        #TODO WE NEED TO BREAK HERE
        #I was thinking we could use the python module threading to 

distance()
#we need to sleep so it doesnt get canceled
time.sleep(1)
for i in range(3):
    cbrise[i].cancel()
    cbfall[i].cancel()