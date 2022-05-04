#!/usr/bin/env python3
import time
import robot
import map

control = robot.EEBot()


N = 0
W = 1
S = 2
E = 3

if __name__ == "__main__":
    try:
        #control.snap90(False)
        #control.snap90(False)
        #control.scan(3)
        #print(*map.map1.shortest_route((0,2), (-2,0)))
        #print(*robot.route_to_directions(map.map1.shortest_route((0,2), (-2,0)),0))
        #print(map.map3)
        route = map.level2.shortest_route(
            map.Intersection(3,0),
            map.Intersection(1,2)
        )
        turns, _ = map.route_to_directions(route, S)
        control.follow_directions(turns)
        #print(map.map1)
        #print()
        #print(map.map2)
        #print()
        #print(map.map3)
        ##m = map.build_map(control, map.Intersection(0,0), 2)
        #print(m)
        #control.turn(3)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    control.shutdown()
    

