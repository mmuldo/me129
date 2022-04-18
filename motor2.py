#!/usr/bin/env python3
# Imports
import pigpio
import sys

MTR1_LEGA = 7
MTR1_LEGB = 8
MTR2_LEGA = 5
MTR2_LEGB = 6

MOTORS = [
    {'A': MTR1_LEGA, 'B': MTR1_LEGB}, 
    {'A': MTR2_LEGA, 'B': MTR2_LEGB}
]

class Motor:
    def __init__(self):
    
        ############################################################
        # Prepare the GPIO connetion (to command the motors).
        print("Setting up the GPIO...")


        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
        # Set up the four pins as output (commanding the motors).
        self.io.set_mode(MTR1_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR1_LEGB, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGB, pigpio.OUTPUT)

        # Prepare the PWM.  The range gives the maximum value for 100%
        # duty cycle, using integer commands (1 up to max).
        self.io.set_PWM_range(MTR1_LEGA, 255)
        self.io.set_PWM_range(MTR1_LEGB, 255)
        self.io.set_PWM_range(MTR2_LEGA, 255)
        self.io.set_PWM_range(MTR2_LEGB, 255)

        # Set the PWM frequency to 1000Hz.  You could try 500Hz or 2000Hz
        # to see whether there is a difference?
        self.io.set_PWM_frequency(MTR1_LEGA, 1000)
        self.io.set_PWM_frequency(MTR1_LEGB, 1000)
        self.io.set_PWM_frequency(MTR2_LEGA, 1000)
        self.io.set_PWM_frequency(MTR2_LEGB, 1000)
        # Clear all pins, just in case.
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)
        print("GPIO ready...")

    def shutdown(self):
        print("Turning off...")

        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)

        self.io.stop()

    def duty_to_pwm(motor, dutycycle):
        magnitude = abs(round(dutycycle*255))
        if magnitude > 255:
            magnitude = 255

        direction = 'A' if dutycycle < 0 else 'B'
        other_direction = 'A' if direction == 'B' else 'B'
        return ((MOTORS[motor-1][direction], magnitude), 
                (MOTORS[motor-1][other_direction], 0))

    def set(self, leftdutycycle, rightdutycycle):
        for motor, duty in enumerate([leftdutycycle, rightdutycycle]):
            pwm = duty_to_pwm(motor+1, duty)

            self.io.set_PWM_dutycycle(*pwm[0])
            self.io.set_PWM_dutycycle(*pwm[1])

    def setlinear(self, speed):
        pwm = 1.9457*speed + 0.1811
        self.set(pwm,pwm)


    # ~ def square(self):
        # ~ # Turn in a box
        # ~ for i in range(4):
            # ~ io.set_PWM_dutycycle(MTR1_LEGA, 0)
            # ~ io.set_PWM_dutycycle(MTR1_LEGB, 220)

            # ~ io.set_PWM_dutycycle(MTR2_LEGA, 0)
            # ~ io.set_PWM_dutycycle(MTR2_LEGB, 200)
            # ~ time.sleep(3)
            
            # ~ io.set_PWM_dutycycle(MTR1_LEGA, 0)
            # ~ io.set_PWM_dutycycle(MTR1_LEGB, 230)

            # ~ io.set_PWM_dutycycle(MTR2_LEGA, 150)
            # ~ io.set_PWM_dutycycle(MTR2_LEGB, 0)
            # ~ time.sleep(.7)
