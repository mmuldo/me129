#!/usr/bin/env python3
import motor
import time

if __name__ == "__main__":
    #initialize the robot
    robo = motor.Motor()

    try:
        robo.setlinear(0.16)
        time.sleep(2)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        
    #shut down robot
    robo.shutdown()
