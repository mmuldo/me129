#!/usr/bin/env python3
import time
import robot
import map

control = robot.EEBot()


if __name__ == "__main__":
    try:
        control.scan(0)
        #print(*map.map1.shortest_route((0,2), (-2,0)))
        #print(*robot.route_to_directions(map.map1.shortest_route((0,2), (-2,0)),0))
        #control.follow_directions(
        #    robot.route_to_directions(
        #        map.map2.shortest_route((3,0),(1,1)),
        #        2
        #    )
        #)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    control.shutdown()
    

