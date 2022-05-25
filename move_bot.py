#!/usr/bin/env python3
import time
import robot
import map
import mapbuilder
import json
import util
import random
import threading 
import demo
import pigpio

passed_io = pigpio.pi()
control = robot.EEBot(passed_io)

N = 0
W = 1
S = 2
E = 3
map_l2 = {}

change_route = [False]


if __name__ == "__main__":

    try:
        #create the map
#         map_l2 = mapbuilder.build_map(control,N,(0,0))
#         json_str = json.dumps(util.map_to_dict(map_l2))
#         with open('map_l2.json', 'w') as file:
#             file.write(json_str)
#         #print(map_l2)
    #load the map

        time.sleep(.5)
        loaded_map = util.load_map('map_l2.json')
        #TODO MATTHEW we need to recall the next function but with our changed route.
        #i suggest that we save changes dynamically to a file such as map_l2_updates.json??
        #that way we dont overwrite our original map, or just as an object?

        #while(1): put the map builder in a while??
        mapbuilder.wrapped_goto(loaded_map, control, change_route, N, (0,0), (1,1))

       # get the bot moving initially
    except BaseException as ex:
       print("Ending due to exception: %s" % repr(ex))

    control.shutdown_ultrasonic()
    control.set_pwm(0,0)
    control.shutdown()
   