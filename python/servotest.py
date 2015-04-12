# Author: Ingmar Stapel 
# Usage: Servo test program
# URL: http://www.raspberry-pi-car.com/top-story-en/raspberry-pi-controlling-servo-motors/7028
# Version: 0.1 beta

import RPi.GPIO as GPIO
import time
import os

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.OUT)

# Now we will start with a PWM signal at 50Hz at pin 11. 
# 50Hz should work for many servos very will. If not you can play with 
# the frequency if you like.	
Servo = GPIO.PWM(11, 50)

while True:
					
	# This command sets the left position of the servo
	Servo.start(2.5)

	# Now the program asks for the direction the servo should turn.
	input = raw_input("Selection: ") 

	if(input == "r"):
		print "Right"
		stepslength = 12.5 / int(10)
		for Counter in range(int(steps)):
			Servo.ChangeDutyCycle(stepslength * (Counter + 1))
			print stepslength * (Counter + 1)
			time.sleep(0.5)
		Servo.stop()

	if(input == "l"):
		print "Left"
		stepslength = 12.5 / int(10)
		for Counter in range(int(steps)):
			Servo.ChangeDutyCycle(stepslength * (Counter + 1))
			print stepslength * (Counter + 1)
			time.sleep(0.5)
		Servo.stop()

	if(input == "s"):
		print "Stop"
		Servo.stop()

	# input not valid
	else:
		print "input not valid!"