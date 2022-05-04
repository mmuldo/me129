#!/usr/bin/env python3
import time
import robot
import map

control = robot.EEBot()


if __name__ == "__main__":
    #try:
        #control.snap90(False)
        #control.snap90(False)
        #control.scan(3)
        #print(*map.map1.shortest_route((0,2), (-2,0)))
        #print(*robot.route_to_directions(map.map1.shortest_route((0,2), (-2,0)),0))
        #print(map.map3)
        #control.follow_directions(
        #    map.route_to_directions(
        #        map.map3.shortest_route(
        #            map.Intersection(2,2),map.Intersection(2,0)
        #        ),
        #        3
        #    )
        #)
        #map.build_map(control, (0,0), 1)
    #except BaseException as ex:
    #    print("Ending due to exception: %s" % repr(ex))

    #control.scan(0)
    m = map.build_map(control, map.Intersection(0,0), 2)
    print(m)
    control.shutdown()
    

