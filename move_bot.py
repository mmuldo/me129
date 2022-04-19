#!/usr/bin/env python3
import motor
import time

robot = motor.Motor()

def drive_shape(turns):
    for i in range(turns):
        robot.setlinear(0.5)
        time.sleep(1)
        robot.setspin_direct(360/turns)

def straight_and_back():
    drive_shape(2)

def triangle():
    drive_shape(3)

def square():
    drive_shape(4)


if __name__ == "__main__":
    try:
        robot.setlinear(0.4)
        time.sleep(0.5)
        #robo.setvel(0.5, 3.14/2)
        #time.sleep(8)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        
    #shut down robot
    robo.shutdown()

