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
direction['u_right'] = 0
direction['u_left'] = 0

desired_distance = .3



def stopcontinual():
    stopflag = True
def runcontinual(stop):
    stopflag = False
    while not stopflag:
        ultra.trigger()

        # print(ultra.distance)

        # e = 0
        # k = 0.3
        # # if the robot is 30cm away, stop
        # if ultra.distance[1] <= desired_distance or (ultra.distance[2] == desired_distance and ultra.distance[0] == desired_distance):
        #     direction['stay_straight'] = True
        #     direction['turn_left'] = False
        #     direction['turn_right'] = False
        #     direction['u'] = 0
        #     print('staying straight')
        # if ultra.distance[2] < desired_distance:
        #     direction['stay_straight'] = False
        #     direction['turn_left'] = True
        #     direction['turn_right'] = False
        #     e = desired_distance - ultra.distance[2]
        #     direction['u'] = -k*e
        #     # print('heading left')
        # if ultra.distance[0] > desired_distance:
        #     direction['stay_straight'] = False
        #     direction['turn_left'] = True
        #     direction['turn_right'] = False
        #     e = desired_distance - ultra.distance[0]
        #     direction['u'] = -k*e
        #     # print('heading left')
        # if ultra.distance[2] > desired_distance:
        #     direction['stay_straight'] = False
        #     direction['turn_left'] = False
        #     direction['turn_right'] = True
        #     e = ultra.distance[2] - desired_distance
        #     direction['u'] = -k*e
        #     # print('heading right')
        # if ultra.distance[0] < desired_distance:
        #     direction['stay_straight'] = False
        #     direction['turn_left'] = False
        #     direction['turn_right'] = True
        #     e = ultra.distance[0] - desired_distance
        #     direction['u'] = -k*e
        #     # print('heading right')
        # else:
        #     direction['stay_straight'] = True
        #     direction['turn_left'] = False
        #     direction['turn_right'] = False
        #     direction['u'] = 0
        #     print('halp')


        # e = 0
        # k = 0.9
        # if the robot is 30cm away, stop
        # if ultra.distance[1] <= desired_distance:
        #     if ultra.distance[0] >= ultra.distance[2]:
        #         while ultra.distance[1] <= desired_distance:
        #             print('YAY')
        #             direction['left'] = True
        #             direction['right'] = False
        #             direction['stop'] = False
        #             e = desired_distance - ultra.distance[0]
        #             direction['u'] = -k*e
        #     elif ultra.distance[0] < ultra.distance[2]:
        #         while ultra.distance[1] <= desired_distance:
        #             print('ok')
        #             direction['right'] = True
        #             direction['left'] = False
        #             direction['stop'] = False
        #             e = ultra.distance[2] - desired_distance
        #             direction['u'] = -k*e
        #     print('stop')
        # elif ultra.distance[0] <= desired_distance:
        #     direction['left'] = True
        #     direction['right'] = False
        #     direction['stop'] = False
        #     e = desired_distance - ultra.distance[0]
        #     direction['u'] = -k*e
        #     print('left')
        # elif ultra.distance[2] <= desired_distance:
        #     direction['right'] = True
        #     direction['left'] = False
        #     direction['stop'] = False
        #     e = ultra.distance[2] - desired_distance
        #     direction['u'] = -k*e
        #     print('right')
        # else:
        #     direction['stop'] = True
        #     direction['left'] = False
        #     direction['right'] = False
        #     direction['u'] = 0


        e_right = 0
        e_left = 0
        k = .1
        # if the robot is 30cm away, stop
        print(ultra.distance)
        if ultra.distance[1] <= desired_distance:
            direction['stop'] = True
            direction['left'] = False
            direction['right'] = False
            direction['u_right'] = 0
            direction['u_left'] = 0
            print('stop')
        elif ultra.distance[2] <= desired_distance:
            direction['right'] = False
            direction['stop'] = False
            direction['left'] = True
            e_right = desired_distance - ultra.distance[0]
            direction['u_right'] = -k*e_right
            # print('right', direction['u_right'])
            # print('R')
        # elif ultra.distance[0] <= desired_distance:
        #     direction['left'] = False
        #     direction['stop'] = False
        #     direction['right'] = True
        #     e_right = desired_distance - ultra.distance[0]
        #     direction['u_right'] = -k*e_right
        #     # print('right', direction['u_right'])
        #     print('R')
        elif ultra.distance[2] > desired_distance:
            direction['left'] = False
            direction['stop'] = False
            direction['right'] = True
            e_left = desired_distance - ultra.distance[2]
            direction['u_left'] = -k*e_left
            # print('left', direction['u_left'])
            # print('L')
        else:
            direction['stop'] = False
            direction['left'] = False
            direction['right'] = False
            direction['u_right'] = 0
            direction['u_left'] = 0

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
            # if direction['stop']:
            #     control.set_pwm(0,0)
            # elif direction['left'] or direction['right']:
            #     u = direction['u']
            #     PWM_left = max(0.5, min(0.9, 0.7 - u))
            #     PWM_right = max(0.5, min(0.9, 0.7 + u))
            #     control.set_pwm(PWM_left,PWM_right)
            # else:
            #     #control.set_pwm(0,0)  
            #     control.set_pwm(.7,.7)   

            # print(direction)
            # print(ultra.distance)
            # if direction['stay_straight']:
            #     # print(ultra.distance)
            #     control.set_pwm(0.7,0.7)
            #     # print('staying straight')
            # elif direction['turn_left']:
            #     u = direction['u']
            #     # PWM_left = max(0.5, min(0.9, 0.7 + u))
            #     # PWM_right = max(0.5, min(0.9, 0.7 - u))
            #     control.set_pwm(0.7+u,0.7-u)
            #     print('heading left')
            #     print(u)
            #     # print('LEFT', PWM_left, PWM_right)
            # elif direction['turn_right']:
            #     u = direction['u']
            #     # PWM_left = max(0.5, min(0.9, 0.7 - u))
            #     # PWM_right = max(0.5, min(0.9, 0.7 + u))
            #     # print('RIGHT', PWM_left, PWM_right)
            #     control.set_pwm(0.7-u,0.7+u)
            #     print('heading right')
            #     print(u)
            # else:
            #     control.set_pwm(0.7,0.7)  
            #     print('smth wack')
            #     print(direction)

            # print(direction['u'])
            # print(ultra.distance)
            # if direction['stop']:
            #     control.set_pwm(0,0)
            # elif direction['left']:
            #     u = direction['u_left']
            #     PWM_left = max(0.5, min(0.9, 0.7 + u))
            #     PWM_right = max(0.5, min(0.9, 0.7 - u))
            #     control.set_pwm(0.8+u,0.8-u+0.1)
            #     print('L', 0.8+u, 0.8-u+0.5)
            #     direction['right'] = False
            #     direction['left'] = False
            # elif direction['right']:
            #     u = direction['u_right']
            #     PWM_left = max(0.5, min(0.9, 0.7 + u))
            #     PWM_right = max(0.5, min(0.9, 0.7 - u))
            #     control.set_pwm(0.8-u+0.1,0.8+u)
            #     print('R', 0.8-u+0.5, 0.8+u)
            #     direction['right'] = False
            #     direction['left'] = False
            # else:
            #     #control.set_pwm(0,0)  
            #     control.set_pwm(0.9,0.9)   


            # herding
            if direction['stop']:
                control.set_pwm(0,0)
                print('staying straight')
            elif direction['left']:
                control.set_pwm(-0.7, 0.7)
                print('heading left')
            elif direction['right']:
                control.set_pwm(0.7, -0.7)
                print('heading right')
            

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    control.shutdown()
    ultra.shutdown_ultrasonic()
    stopcontinual()
    thread.join()