import numpy as np
import cv2
import sys
import lcd

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

cv2.namedWindow('result')

_redL = np.array([1,150,60])
_redH = np.array([33,255,255])

def gpioInit():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LED_GREEN, GPIO.OUT)  
	GPIO.setup(LED_RED, GPIO.OUT) 
	GPIO.setup(SWITCH_CAPTURE, GPIO.IN, pull_up_down = GPIO.PUD_UP)  
	GPIO.setup(SWITCH_STOP, GPIO.IN, pull_up_down = GPIO.PUD_UP) 
	GPIO.output(LED_RED, True)
	GPIO.output(LED_GREEN, True)

def run():
	_frame 	= cv2.imread("1.jpg",-1)
	frame 	= np.zeros((640,480,3), np.uint8)
	frame	= cv2.resize(frame, (640,480))
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
	__x		= moments['m10']
	__y		= moments['m01']

	X 		= __x / area
	Y 		= __y / area

	print "X : " + str(X)
	print "Y : " + str(Y)
	print "A : " + str(area)

	if area > 2500000:
		cv2.circle(frame,(int(X),int(Y)), int((area / 100000)), (22,68,196), 2)
		lcd.lcd_byte(0x94, False)
		lcd.lcd_string("Loc(x,y): ")
		lcd.lcd_string(str(int(X)) + ", " + str(int(Y)))
		lcd.lcd_string("    ")
		lcd.lcd_byte(0xD4, False)
		lcd.lcd_string("radius: ")
		lcd.lcd_string(str(int((area / 100000))))
		lcd.lcd_string("    ")
	cv2.imshow("result",frame)

# Init servo
os.system("servod --p1pins=7,0,0,0,0,0,0,0")
os.system("echo 0=50% > /dev/servoblaster")

# Init lcd
lcd = LCD(LCD_RS, LCD_E, LCD_D4, LCD_D5,LCD_D6,LCD_D7)
lcd.lcd_byte(0x80, False)
lcd.lcd_string("  Color Detection  ")

while True:
	if GPIO.input(SWITCH_CAPTURE) 	== 0:
		lcd.lcd_byte(0xC0, False)
		lcd.lcd_string("Capturing         ")
		run()
		print "Capture"
	if GPIO.input(SWITCH_STOP) 		== 0:
		lcd.lcd_string("Clearing          ")
		print "Clear"

	k = cv.waitKey(5) & 0xFF
	if k == 27:
		break



