#!/usr/bin/env python3
import time
import robot
import map
import mapbuilder
import json
import util
import ultrasonic
import random
import threading 

control = robot.EEBot()
ultra = ultrasonic.Ultrasonic()

N = 0
W = 1
S = 2
E = 3
map_l2 = {}

desired_distance = 0.3


def stopcontinual():
    stopflag = True
def runcontinual(stop):
    stopflag = False
    while not stopflag:
        ultra.trigger()
        time.sleep(.08 + 0.04 * random.random())
        print(ultra.distance)
        

if __name__ == "__main__":
    #start ultrasonic threading
    thread = threading.Thread(target=runcontinual,args=(direction,))
    thread.start()  

    try:
        #create the map
#         map_l2 = mapbuilder.build_map(control,N,(0,0))
#         json_str = json.dumps(util.map_to_dict(map_l2))
#         with open('map_l2.json', 'w') as file:
#             file.write(json_str)
#         #print(map_l2)

        # #load the map
        # with open('map_l2.json', 'r') as file:
        #     data = file.read()
        # loaded_map = json.loads(data)
        # #print(util.dict_to_map(loaded_map))


       # get the bot moving initially
        control.set_pwm(0.8,0.8)
        time.sleep(0.5)
        while(1):
            e = 0
            k = .3
            u = 0
            # if the robot is at desired distance, stop
            if ultra.distance[1] <= desired_distance:
                control.set_pwm(0,0)
                # print('stop')
            # if the robot is within the desired distance on the right,
            # turn left
            elif ultra.distance[2] <= desired_distance:
                e = desired_distance - ultra.distance[2]
                u = k*e
                PWM_left = max(0.6, min(0.9, 0.8 - u))
                PWM_right = max(0.6, min(0.9, 0.8 + u))
                control.set_pwm(PWM_left,PWM_right)
                # print('left')
            # if the robot is within the desired distance on the left,
            # turn right
            elif ultra.distance[2] > desired_distance:
                e = ultra.distance[2] - desired_distance
                u = -k*e
                PWM_left = max(0.6, min(0.9, 0.8 - u))
                PWM_right = max(0.6, min(0.9, 0.8 + u))
                control.set_pwm(PWM_left,PWM_right)
                # print('right')
            # otherwise, stay straight
            else:
                control.set_pwm(0.8,0.8)
                # print('stay straight')
            
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    control.shutdown()
    ultra.shutdown_ultrasonic()
    stopcontinual()
    thread.join()