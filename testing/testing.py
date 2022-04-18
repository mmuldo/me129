#!/usr/bin/env python
#
#   This file is Hope's attempt to make sense of this robot.
#   The current conditions for when the bot will actually make a square
#   are "slightly dusty wheels" and running this on the floor :')




# Imports
import pigpio
import sys
import time
# Define the motor pins.
MTR1_LEGA = 7
MTR1_LEGB = 8
MTR2_LEGA = 5
MTR2_LEGB = 6
#
#   Main
#
if __name__ == "__main__":
    ############################################################
    # Prepare the GPIO connetion (to command the motors).
    print("Setting up the GPIO...")
    
    # Initialize the connection to the pigpio daemon (GPIO interface).
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    # Set up the four pins as output (commanding the motors).
    io.set_mode(MTR1_LEGA, pigpio.OUTPUT)
    io.set_mode(MTR1_LEGB, pigpio.OUTPUT)
    io.set_mode(MTR2_LEGA, pigpio.OUTPUT)
    io.set_mode(MTR2_LEGB, pigpio.OUTPUT)
    # Prepare the PWM.  The range gives the maximum value for 100%
    # duty cycle, using integer commands (1 up to max).
    io.set_PWM_range(MTR1_LEGA, 255)
    io.set_PWM_range(MTR1_LEGB, 255)
    io.set_PWM_range(MTR2_LEGA, 255)
    io.set_PWM_range(MTR2_LEGB, 255)
    
    # Set the PWM frequency to 1000Hz.  You could try 500Hz or 2000Hz
    # to see whether there is a difference?
    io.set_PWM_frequency(MTR1_LEGA, 100)
    io.set_PWM_frequency(MTR1_LEGB, 100)
    io.set_PWM_frequency(MTR2_LEGA, 100)
    io.set_PWM_frequency(MTR2_LEGB, 100)
    # Clear all pins, just in case.
    io.set_PWM_dutycycle(MTR1_LEGA, 0)
    io.set_PWM_dutycycle(MTR1_LEGB, 0)
    io.set_PWM_dutycycle(MTR2_LEGA, 0)
    io.set_PWM_dutycycle(MTR2_LEGB, 0)
    print("GPIO ready...")
    ############################################################
    try:
        # Trying to understand what leg A vs B do
        for i in range(0, 4):
            print("Driving straight")
            io.set_PWM_dutycycle(MTR1_LEGA, 0)
            io.set_PWM_dutycycle(MTR1_LEGB, 225)
            io.set_PWM_dutycycle(MTR2_LEGA,   0)
            io.set_PWM_dutycycle(MTR2_LEGB,   215)
            time.sleep(2)
            io.set_PWM_dutycycle(MTR1_LEGA,   0)
            io.set_PWM_dutycycle(MTR1_LEGB,   0)        
            io.set_PWM_dutycycle(MTR2_LEGA,   0)
            io.set_PWM_dutycycle(MTR2_LEGB,   0)
            time.sleep(1)        
            print("Turning 90 degrees")
            io.set_PWM_dutycycle(MTR1_LEGA,   0)
            io.set_PWM_dutycycle(MTR1_LEGB,   225)
            io.set_PWM_dutycycle(MTR2_LEGA,   225)
            io.set_PWM_dutycycle(MTR2_LEGB,   0)
            time.sleep(0.93)
            io.set_PWM_dutycycle(MTR1_LEGA,   0)
            io.set_PWM_dutycycle(MTR1_LEGB,   0)        
            io.set_PWM_dutycycle(MTR2_LEGA,   0)
            io.set_PWM_dutycycle(MTR2_LEGB,   0)
            time.sleep(1)          
        ############################################################
        # Turn Off.
        # Note the PWM still stay at the last commanded value.  So you
        # want to be sure to set to zero before the program closes.  else
        # your robot will run away...
        print("Turning off...")
        # Clear the PINs (commands).
        io.set_PWM_dutycycle(MTR1_LEGA, 0)
        io.set_PWM_dutycycle(MTR1_LEGB, 0)
        io.set_PWM_dutycycle(MTR2_LEGA, 0)
        io.set_PWM_dutycycle(MTR2_LEGB, 0)
        
        # Also stop the interface.
        io.stop()
        
    except BaseException:
        print("Turning off...")
        # Clear the PINs (commands).
        io.set_PWM_dutycycle(MTR1_LEGA, 0)
        io.set_PWM_dutycycle(MTR1_LEGB, 0)
        io.set_PWM_dutycycle(MTR2_LEGA, 0)
        io.set_PWM_dutycycle(MTR2_LEGB, 0)
        
        # Also stop the interface.
        io.stop()

