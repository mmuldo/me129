#!/usr/bin/env python3
import time
import robot
import map
import mapbuilder
import json
import util

control = robot.EEBot()


N = 0
W = 1
S = 2
E = 3
map_l2 = {}

if __name__ == "__main__":
    try:
        #create the map
#         map_l2 = mapbuilder.build_map(control,N,(0,0))
#         json_str = json.dumps(util.map_to_dict(map_l2))
#         with open('map_l2.json', 'w') as file:
#             file.write(json_str)
#         #print(map_l2)

        #load the map
        with open('map_l2.json', 'r') as file:
            data = file.read()
        loaded_map = json.loads(data)
        #print(util.dict_to_map(loaded_map))
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    control.shutdown()
    

