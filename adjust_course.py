import robot
import ultrasonic


desired_distance = 0.3

class Adjust:
    def __init__(self):
        pass
    def emergency(self, control, ultra):
        # e = 0
        # k = .3
        # u = 0
        # if the robot is at desired distance, stop
        if ultra.distance[1] <= desired_distance:
            #control.set_pwm(0,0)
            print('stop')
        # if the robot is within the desired distance on the right,
        # turn left
        # elif ultra.distance[2] <= desired_distance:
        #     e = desired_distance - ultra.distance[2]
        #     u = k*e
        #     PWM_left = max(0.6, min(0.9, 0.8 - u))
        #     PWM_right = max(0.6, min(0.9, 0.8 + u))
        #     control.set_pwm(PWM_left,PWM_right)
        #     # print('left')
        # # if the robot is within the desired distance on the left,
        # # turn right
        # elif ultra.distance[2] > desired_distance:
        #     e = ultra.distance[2] - desired_distance
        #     u = -k*e
        #     PWM_left = max(0.6, min(0.9, 0.8 - u))
        #     PWM_right = max(0.6, min(0.9, 0.8 + u))
        #     control.set_pwm(PWM_left,PWM_right)
            # print('right')


