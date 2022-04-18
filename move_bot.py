#!/usr/bin/env python3
import motor
import time

def drive_shape(turns):
    for i in range(turns):
        robo.setlinear(0.5)
        time.sleep(1)
        robo.setspin(360/turns)

def straight_and_back():
    drive_shape(2)

def triangle():
    drive_shape(3)

def square():
    drive_shape(4)

if __name__ == "__main__":
    #initialize the robot
    robo = motor.Motor()

    try:
        triangle()
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        
    #shut down robot
    robo.shutdown()

