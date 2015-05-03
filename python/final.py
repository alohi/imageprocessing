import numpy as np
import cv2
import os
import sys
from lcd import LCD
from imthread import CapThread
import threading
import time
import RPi.GPIO as GPIO
import math

_threshold = 27000000
_div = 100000

ui_enable = False

# LCD Pins
LCD_RS 			= 7
LCD_E  			= 8
LCD_D4 			= 25
LCD_D5 			= 24
LCD_D6 			= 23
LCD_D7 			= 18

# GPIO Pins
SWITCH_CAPTURE 	= 11
SWITCH_STOP	= 9

LED_GREEN = 22
LED_RED = 10

gud = 0
tot = 0

if ui_enable == True:
        cv2.namedWindow('result')

_redL = np.array([1,150,60])
_redH = np.array([33,255,255])

def l_map(x, in_min, in_max, out_min, out_max): 
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def gpioInit():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LED_GREEN, GPIO.OUT)  
	GPIO.setup(LED_RED, GPIO.OUT) 
	GPIO.setup(SWITCH_CAPTURE, GPIO.IN, pull_up_down = GPIO.PUD_UP)  
	GPIO.setup(SWITCH_STOP, GPIO.IN, pull_up_down = GPIO.PUD_UP) 
	GPIO.output(LED_RED, True)
	GPIO.output(LED_GREEN, True)

def run():
        print 'capturing'
        os.system('fswebcam -r 1284x720 -d /dev/video0 --no-banner /tmp/test.jpg -S 30')
        time.sleep(2)
        print 'processing'
        lcd.lcd_byte(0x80, False)
	lcd.lcd_string("Processing       ")
        frame = cv2.imread('/tmp/test.jpg')
        frame 	= cv2.GaussianBlur(frame, (5,5),0)
        hsv 	= cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask	= cv2.inRange(hsv,_redL,_redH)

        se1 	= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
        se2 	= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(1,1))

        kernel 	= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        mask 	= cv2.dilate(mask,kernel,iterations = 1)

        mask 	= cv2.morphologyEx(mask, cv2.MORPH_CLOSE, se1)
        mask 	= cv2.morphologyEx(mask, cv2.MORPH_OPEN, se2)

        moments = cv2.moments(mask)
        area	= moments['m00']

        print 'area: ' + str(area)
        rad = math.sqrt(area/3.142)
        print 'radius: ' + str(rad)

        if area > _threshold:
                lcd.lcd_byte(0xC0, False)
                lcd.lcd_string("Good One          ")
                __x	= moments['m10']
                __y	= moments['m01']

                X 	= __x / area
                Y 	= __y / area

                print "X : " + str(X)
                print "Y : " + str(Y)
                print "A : " + str(area)
                if ui_enable:
                        cv2.circle(frame,(int(X),int(Y)), int((area / _div)), (22,68,196), 2)
                        cv2.imshow('result', frame)
		per = int(l_map(area, 20000000, 50000000, 70,95))
                lcd.lcd_byte(0xD4, False)
                lcd.lcd_string("Loc(x,y): ")
                str1 = str(int(X)) + ", " + str(int(Y))
                lcd.lcd_string(str1)
                lcd.lcd_string("    ")
                lcd.lcd_byte(0x94, False)
                lcd.lcd_string("per: ")
		#lcd.lcd_string(str(int(per))))
                lcd.lcd_string(str(per))
                lcd.lcd_string("    ")
                GPIO.output(LED_RED, False)
                GPIO.output(LED_GREEN, True)
		#per = l_map(area, 20000000, 50000000, 0, 100)
		#print str(per)
                os.system("echo 0=100% > /dev/servoblaster")
                time.sleep(2)
                os.system("echo 0=50% > /dev/servoblaster")
                return True
        else:
		per = int(l_map(area, 0, 20000000, 0,30))
                lcd.lcd_byte(0x80, False)
                lcd.lcd_string("Bad One          ")
                if ui_enable == True:
                        cv2.imshow('result', frame)
                GPIO.output(LED_RED, True)
                GPIO.output(LED_GREEN, False)
		lcd.lcd_byte(0x94, False)
		lcd.lcd_string("per:")
		lcd.lcd_string(str(per))
		lcd.lcd_string("    ")
                os.system("echo 0=0% > /dev/servoblaster")
                time.sleep(2)
                os.system("echo 0=50% > /dev/servoblaster")
                return False
           
        
GPIO.setwarnings(False)

# init gpio
gpioInit()

# Init servo
os.system("servod --p1pins=7,0,0,0,0,0,0,0")
os.system("echo 0=50% > /dev/servoblaster")

# Init lcd
lcd = LCD(LCD_RS, LCD_E, LCD_D4, LCD_D5,LCD_D6,LCD_D7)
#lcd.lcd_byte(0x80, False)
#lcd.lcd_string("  Color Detection  ")

#run()

lcd.lcd_byte(0xC0, False)
lcd.lcd_string("Ready......     ")

while True:
	if GPIO.input(SWITCH_CAPTURE) 	== 0:
		lcd.lcd_byte(0x80, False)
		lcd.lcd_string("Capturing       ")
		res = run()
		lcd.lcd_byte(0x80, False)
		lcd.lcd_string("Completed       ")
		if res == True:
                    gud += 1
                tot += 1
		lcd.lcd_byte(0xC0, False)
		lcd.lcd_string('red:')
		lcd.lcd_string(str(gud))
		lcd.lcd_string('    ')
		lcd.lcd_string('n-red: ')
		lcd.lcd_string(str(tot - gud))
		lcd.lcd_string('    ')
		lcd.lcd_byte(0x80, False)
		lcd.lcd_string('total:')
		lcd.lcd_string(str(tot))
		lcd.lcd_string('      ')
		print "Capture"
	if GPIO.input(SWITCH_STOP) 		== 0:
                tot = 0
                gud = 0
                lcd.lcd_byte(0x80, False)
		lcd.lcd_string("Clearing        ")
		lcd.lcd_byte(0xC0, False)
		lcd.lcd_string('red:')
		lcd.lcd_string(str(gud))
		lcd.lcd_string('    ')
		lcd.lcd_string('n-red:')
		lcd.lcd_string(str(tot - gud))
		lcd.lcd_string('    ')
		lcd.lcd_byte(0x80, False)
		lcd.lcd_string('total:')
		lcd.lcd_string(str(tot))
		lcd.lcd_string('    ')
		print "Clear"

	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

        



