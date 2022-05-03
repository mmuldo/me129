#!/usr/bin/env python3
import time
import robot
import map

control = robot.EEBot()


if __name__ == "__main__":
    try:
        control.follow_directions(
            robot.route_to_directions(
                map.map1.shortest_route((0,0),(-3,1)), 
                0
            )
        )
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    control.shutdown()
    

