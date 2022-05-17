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
stop = [False]

def stopcontinual():
    stopflag = True
def runcontinual(stop):
    stopflag = False
    while not stopflag:
        ultra.trigger()
        # if the robot is 20cm away, stop
        if ultra.distance[1] <= .2:
            stop[0] = True
        else:
            stop[0] = False
        time.sleep(0.8 + 0.4 * random.random())
        

if __name__ == "__main__":
    #start ultrasonic threading
    thread = threading.Thread(target=runcontinual,args=(stop,))
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
        while(1):
            print(stop[0])
            if stop[0]:
                control.set_pwm(0,0)
            else:
                control.set_pwm(.6,.6)     
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    control.shutdown()
    ultra.shutdown_ultrasonic()
    stopcontinual()
    thread.join()