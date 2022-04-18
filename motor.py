#!/usr/bin/env python3
# Imports
import pigpio
import sys

MTR1_LEGA = 7
MTR1_LEGB = 8
MTR2_LEGA = 5
MTR2_LEGB = 6
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

	def setlinear(self, speed):
		pass
	def set(self, leftdutycycle, rightdutycycle):
		left = leftdutycycle
		right = rightdutycycle
		
		dir_left = None
		dir_right = None
		
		#make sure magnitude <= 1
		if abs(left) > 1:
			left /= abs(left)
		
		if abs(right) > 1:
			right /= abs(right)
		
		#find which direction
		if right < 0:
			dir_right = 'A'
		else:
			dir_right = 'B'
			
		if left < 0:
			dir_left = 'A'
		else:
			dir_left = 'B'
			
		#adjust to PWM values
		right = abs(round(right*255))
		left = abs(round(left*255))
		
		#set PWM values

		if dir_left == 'A':
			self.io.set_PWM_dutycycle(MTR1_LEGA, left)
			self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
		
		elif dir_left == 'B':
			self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
			self.io.set_PWM_dutycycle(MTR1_LEGB, left)
			
		if dir_right == 'A':
			self.io.set_PWM_dutycycle(MTR2_LEGA, right)
			self.io.set_PWM_dutycycle(MTR2_LEGB, 0)
		
		elif dir_left == 'B':
			self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
			self.io.set_PWM_dutycycle(MTR2_LEGB, right)
			
		
			
			
		
	
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

		
