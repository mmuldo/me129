#!/usr/bin/env python3
# Imports
import pigpio
import sys
import time

from typing import Tuple

MTR1_LEGA = 7
MTR1_LEGB = 8
MTR2_LEGA = 5
MTR2_LEGB = 6

# motor1: left wheel, motor2: right wheel
# A: negative terminal, B: positive terminal
MOTORS = [
    {'A': MTR1_LEGA, 'B': MTR1_LEGB},
    {'A': MTR2_LEGA, 'B': MTR2_LEGB}
]

MOTOR_LEGS = [MTR1_LEGA, MTR1_LEGB, MTR2_LEGA, MTR2_LEGB]


def duty_to_pwm(motor_number: int, dutycycle: float) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    '''
    converts dutycycle to pwm

    Parameters
    ----------
    motor_number : int
        1 -> for left motor, 2 -> for right motor; errors if anything else
    dutycycle : float
        dutycycle, between -1.0 and 1.0

    Returns
    -------
    Tuple[Tuple[int, int], Tuple[int, int]]
        two tuples: the first is the (motor_leg, pwm) for the leg of the motor being set and the second
        is (other_motor_leg, 0)
    '''
    magnitude = abs(round(dutycycle*255))
    if magnitude > 255:
        # max pwm is 255
        magnitude = 255

    # negative dutycycle -> leg A, positive -> leg B
    direction = 'A' if dutycycle < 0 else 'B'
    other_direction = 'A' if direction == 'B' else 'B'

    return ((MOTORS[motor_number-1][direction], magnitude),
            (MOTORS[motor_number-1][other_direction], 0))

class Motor:
    '''
    abstract representation of robot motor, comprised of left and right motors
    each with a positive and negative terminals (i.e. each motor can spin cw and ccw)

    Attributes
    ----------
    io
        pigpio instance

    Methods
    -------
    shutdown()
        stops robot
    set(leftdutycycle, rightdutycycle)
        sets pwm of left and right motors
    setlinear(speed)
        sets linear speed (m/s) of robot
    setspin(w)
        sets angular speed (degrees/s) of robot
    setspin_direct(theta)
        spins robot a fixed amount of degrees
    '''
    def __init__(self):

        ############################################################
        # Prepare the GPIO connetion (to command the motors).
        print("Setting up the GPIO...")

        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)

        for motor_leg in MOTOR_LEGS:
            # Set up the four pins as output (commanding the motors).
            self.io.set_mode(motor_leg, pigpio.OUTPUT)

            # Prepare the PWM.  The range gives the maximum value for 100%
            # duty cycle, using integer commands (1 up to max).
            self.io.set_PWM_range(motor_leg, 255)

            # Set the PWM frequency to 1000Hz.  You could try 500Hz or 2000Hz
            # to see whether there is a difference?
            self.io.set_PWM_frequency(motor_leg, 1000)

            # Clear all pins, just in case.
            self.io.set_PWM_dutycycle(motor_leg, 0)
        print("GPIO ready...")

    def shutdown(self):
        '''stops robot'''
        print("Turning off...")

        for motor_leg in MOTOR_LEGS:
            self.io.set_PWM_dutycycle(motor_leg, 0)

        self.io.stop()


    def set(self, leftdutycycle: float, rightdutycycle: float):
        '''
        sets the PWM for both motors

        Parameters
        ----------
        leftdutycycle
            dutycycle for left motor, between -1.0 and 1.0
        rightdutycycle
            dutycycle for left motor, between -1.0 and 1.0
        '''
        for i, dutycycle in enumerate([leftdutycycle, rightdutycycle]):
            motor_number = i + 1
            motor_pwm = duty_to_pwm(motor_number, dutycycle)

            self.io.set_PWM_dutycycle(*motor_pwm[0])
            self.io.set_PWM_dutycycle(*motor_pwm[1])

    def setlinear(self, speed):
        '''
        sets linear velocity of robot in m/s
        '''
        #pwm = 1.9457*speed + 0.1811
        pwm = 1.57*speed + 0.324
        self.set(pwm,pwm)

    def setspin(self, w):
        '''
        sets angular velocity of robot in degrees/s
        positive w -> cw spin
        negative w -> ccw spin
        '''
        #pwm = 1.9457*speed + 0.1811
        sign = -1 if w < 0 else 1
        pwm = sign*(2.42*abs(w) + 0.447)
        self.set(pwm,-pwm)

    def setspin_direct(self, theta):
        '''
        rotates robot by theta degrees
        '''
        sign = -1 if theta < 0 else 1
        omega = .58
        t = abs(theta/90)
        self.set(sign*omega, -sign*omega)
        time.sleep(t)
