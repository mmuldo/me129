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

direction = {}
direction['left'] = False
direction['right'] = False
direction['stop'] = False
direction['u'] = 0

desired_distance = .3



def stopcontinual():
    stopflag = True
def runcontinual(stop):
    stopflag = False
    while not stopflag:
        ultra.trigger()

        e = 0
        k = .5
        # if the robot is 30cm away, stop
        if ultra.distance[1] <= desired_distance:
            direction['stop'] = True
            direction['left'] = False
            direction['right'] = False
            direction['u'] = 0
            print('stop')
        elif ultra.distance[0] <= desired_distance:
            direction['left'] = True
            e = desired_distance - ultra.distance[0]
            direction['u'] = -k*e
            print('left')
        elif ultra.distance[2] <= desired_distance:
            direction['right'] = True
            e = ultra.distance[2] - desired_distance
            direction['u'] = -k*e
            print('right')
        else:
            direction['stop'] = False
            direction['left'] = False
            direction['right'] = False
            direction['u'] = 0


        time.sleep(.4 + 0.4 * random.random())
        

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


       #u positive is left, negative is right
        while(1):
            print(direction['u'])
            if direction['stop']:
                control.set_pwm(0,0)
            elif direction['left'] or direction['right']:
                u = direction['u']
                PWM_left = max(0.5, min(0.9, 0.7 - u))
                PWM_right = max(0.5, min(0.9, 0.7 + u))
                control.set_pwm(PWM_left,PWM_right)
            else:
                #control.set_pwm(0,0)  
                control.set_pwm(.7,.7)     
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    control.shutdown()
    ultra.shutdown_ultrasonic()
    stopcontinual()
    thread.join()