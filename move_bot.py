#!/usr/bin/env python3
import time
import robot
import map
import mapbuilder
import json
import util
# import ultrasonic
import random
import threading 
import adjust_course
import demo
import pigpio

passed_io = pigpio.pi()
control = robot.EEBot(passed_io)
# ultra = ultrasonic.Ultrasonic(passed_io)

N = 0
W = 1
S = 2
E = 3
map_l2 = {}

# stopflag = False
# stop_ultra = False
change_route = [False]

# def stopcontinual():
#     global stopflag
#     stopflag = True

# def runcontinual():
#     global stopflag
#     stopflag = False
#     while not stopflag:
#         ultra.trigger()
#         time.sleep(.08 + 0.04 * random.random())

# def stopUltra():
#     global stop_ultra
#     stop_ultra = True
    
# def runUltra():
#     global stop_ultra
#     stop_ultra = False
#     adjustments = adjust_course.Adjust()
#     while not stop_ultra:
#         time.sleep(0.01)
#         adjustments.emergency(control, ultra, change_route)
       

if __name__ == "__main__":
    #start ultrasonic threading
    # thread = threading.Thread(target=runcontinual,name='runcontinual')
    # thread.start()
    # time.sleep(.5)
    # threadUltra = threading.Thread(target=runUltra,name='ultrasonic')
    # threadUltra.start()

    #try:
        #create the map
#         map_l2 = mapbuilder.build_map(control,N,(0,0))
#         json_str = json.dumps(util.map_to_dict(map_l2))
#         with open('map_l2.json', 'w') as file:
#             file.write(json_str)
#         #print(map_l2)
    #load the map
    time.sleep(.5)
    loaded_map = util.load_map('map_l2.json')
    mapbuilder.wrapped_goto(loaded_map, control, change_route, N, (0,0), (1,1))

       # get the bot moving initially
    #except BaseException as ex:
    #    print("Ending due to exception: %s" % repr(ex))

    control.shutdown_ultrasonic()
    control.set_pwm(0,0)
    control.shutdown()
    # stopcontinual()
    # thread.join()
    # stopUltra()
    # threadUltra.join()

