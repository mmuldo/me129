#!/usr/bin/env python3
import motor
import time

if __name__ == "__main__":
    #initialize the robot
    robo = motor.Motor()

    try:
        # Moving robot linearly
#         robo.setlinear(0.16)
#         time.sleep(2)
        # Testing spinning
        # robo.set(0.6, -0.6)
        robo.setspin(180)
        #time.sleep(1)
        
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        
    #shut down robot
    robo.shutdown()
