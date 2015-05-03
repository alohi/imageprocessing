import numpy as np
import cv2
import os
import sys
from lcd import LCD
from imthread import CapThread
import threading
import time
import RPi.GPIO as GPIO

# LCD Pins
LCD_RS 			= 7
LCD_E  			= 8
LCD_D4 			= 25
LCD_D5 			= 24
LCD_D6 			= 23
LCD_D7 			= 18

# GPIO Pins
SWITCH_CAPTURE 	= 11
SWITCH_STOP		= 9

LED_GREEN = 10
LED_RED = 22

cv2.namedWindow('result')

_redL = np.array([1,150,60])
_redH = np.array([33,255,255])

##capture = cv2.VideoCapture(-1)

def gpioInit():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LED_GREEN, GPIO.OUT)  
	GPIO.setup(LED_RED, GPIO.OUT) 
	GPIO.setup(SWITCH_CAPTURE, GPIO.IN, pull_up_down = GPIO.PUD_UP)  
	GPIO.setup(SWITCH_STOP, GPIO.IN, pull_up_down = GPIO.PUD_UP) 
	GPIO.output(LED_RED, True)
	GPIO.output(LED_GREEN, True)

#def run():
	#_frame 	= cv2.imread("1.jpg",-1)
##        if capture.isOpened():
##                capture.close()
##                
##        capture.open(-1)
##        if capture.isOpened():
##                time.sleep(2)
##        ret, _frame = capture.read()
##        if ret == True:
##                print _frame.shape
##                if 
##        capture.release()
##        
##        if capture.isOpened():
##                print 'opened'
##                ret, _frame = capture.read()
##                print type(_frame)
##                print ret
##
##        else:
##                print 'closed'
##                capture.open(-1)
##                time.sleep(1)
##                if capture.isOpened():
##                         print 'opened now'
                
                
        
##        cv2.imshow("result",_frame)
##                capture.close()
	
	#print _frame
	#cv2.imshow("result",_frame)
##	frame 	= np.zeros((640,480,3), np.uint8)
##	frame	= cv2.resize(_frame, (640,480))
##	frame 	= cv2.GaussianBlur(frame, (5,5),0)
##	hsv 	= cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
##	mask	= cv2.inRange(hsv,_redL,_redH)
##
##	se1 	= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
##	se2 	= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(1,1))
##
##	kernel 	= cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
##	mask 	= cv2.dilate(mask,kernel,iterations = 1)
##
##	mask 	= cv2.morphologyEx(mask, cv2.MORPH_CLOSE, se1)
##	mask 	= cv2.morphologyEx(mask, cv2.MORPH_OPEN, se2)
##
##	moments = cv2.moments(mask)
##	area	= moments['m00']
##
##        print 'area' + str(area)
##
##	if area > 2500000:
##                __x	= moments['m10']
##                __y	= moments['m01']
##
##                X 	= __x / area
##                Y 	= __y / area
##
##                print "X : " + str(X)
##                print "Y : " + str(Y)
##                print "A : " + str(area)
##		cv2.circle(frame,(int(X),int(Y)), int((area / 100000)), (22,68,196), 2)
##		lcd.lcd_byte(0x94, False)
##		lcd.lcd_string("Loc(x,y): ")
##		str1 = str(int(X)) + ", " + str(int(Y))
##		lcd.lcd_string(str1)
##		lcd.lcd_string("    ")
##		lcd.lcd_byte(0xD4, False)
##		lcd.lcd_string("radius: ")
##		lcd.lcd_string(str(int((area / 100000))))
##		lcd.lcd_string("    ")

##GPIO.setwarnings(False)
##
### init gpio
##gpioInit()
##
### Init servo
##os.system("servod --p1pins=7,0,0,0,0,0,0,0")
##os.system("echo 0=50% > /dev/servoblaster")
##
### Init lcd
##lcd = LCD(LCD_RS, LCD_E, LCD_D4, LCD_D5,LCD_D6,LCD_D7)
##lcd.lcd_byte(0x80, False)
##lcd.lcd_string("  Color Detection  ")
####time.sleep(2)
##
##run()
##
##while True:
##	if GPIO.input(SWITCH_CAPTURE) 	== 0:
##		lcd.lcd_byte(0xC0, False)
##		lcd.lcd_string("Capturing       ")
##		run()
##		lcd.lcd_byte(0xC0, False)
##		lcd.lcd_string("Completed       ")
##		print "Capture"
##	if GPIO.input(SWITCH_STOP) 		== 0:
##                lcd.lcd_byte(0xC0, False)
##		lcd.lcd_string("Clearing        ")
##		print "Clear"
##
##	k = cv2.waitKey(5) & 0xFF
##	if k == 27:
##		break

##def main():
##        thread_lock = threading.Lock()
##        thread1 = CapThread(1, 'capture', -1, thread_lock)
##        thread1.start()
##        key = cv2.waitKey(0)
##        if key == 27:
##                quit()
##
##if __name__ == '__main__':
##        main()

##def diffImg(t0, t1, t2):
##        d1 = cv2.absdiff(t2,t1)
##        d2 = cv2.absdiff(t1,t0)
##        return cv2.bitwise_and(d1, d2)
##
##cap = cv2.VideoCapture(-1)
##ret, t_minus = cap.read()
##ret, t = cap.read()
##ret, t_plus = cap.read()
##
##print t_minus.shape
##print t.shape
##print t_plus.shape

##def mse(imageA,imageB):
##        err = np.sum((imageA.astype('float') - imageB.astype('float')) ** 2)
##        err /= float(imageA.shape[0] * imageA.shape[1])
##        return err
##
##def compare_images(imageA, imageB):
##        m = mse(imageA,imageB)
##        s = ssim(imageA, imageB)
##        print m
##        print s

cap   = cv2.VideoCapture(-1)

_, first = cap.read()

while True:
        ret, frame = cap.read()
        if ret == True:
##                res = cv2.absdiff(frame,first)
                d1 = cv2.absdiff(first,frame)
                print d1[0,0]
##        time.sleep(1)
##        if ret == True:
##                t_minus = t
##                t = t_plus
##                diffImg(t_minus,t,t_plus).shape
        



