#!/usr/bin/env python3
import time
import robot

control = robot.EEBot()


if __name__ == "__main__":
    try:
        control.turn(-1)
        #control.known_map()
        #control.follow_tape()
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    control.shutdown()
    

