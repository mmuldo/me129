#!/usr/bin/env python3
import motor
import time
import robot

#old_motor = motor.Motor()
control = robot.EEBot()

# def drive_shape(turns):
#     for i in range(turns):
#         old_motor.setlinear(0.5)
#         time.sleep(1)
#         old_motor.setspin_direct(360/turns)
# 
# def straight_and_back():
#     drive_shape(2)
# 
# def triangle():
#     drive_shape(3)
# 
# def square():
#     drive_shape(4)


if __name__ == "__main__":
    try:
        while(1):
            control.detectors_status()
            time.sleep(1)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    #shutdown robot
    control.shutdown()
    #shut down old_motor
    #old_motor.shutdown()

