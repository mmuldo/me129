#!/usr/bin/env python3
import time
import robot

control = robot.EEBot()


if __name__ == "__main__":
    try:
        #streets = control.check_intersection()
        #print(streets)
        control.turn(2)
#        time.sleep(2.5)
 #       control.turn(1)
  #      time.sleep(2.5)
   #     control.turn(2)
    #    time.sleep(2.5)
     #   control.turn(-1)
      #  time.sleep(2.5)
        #control.known_map()
        #control.follow_tape()
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    control.shutdown()
    

