
import cv2 as cv
import numpy as np
import time
import sys
import os

lcdEnable = True
gpioEnable = True

if lcdEnable == True or gpioEnable == True:
	import RPi.GPIO as GPIO

# Define GPIO to LCD mapping
LCD_RS = 7
LCD_E  = 8
LCD_D4 = 25
LCD_D5 = 24
LCD_D6 = 23
LCD_D7 = 18

SWITCH_CAPTURE 	= 11
SWITCH_STOP		= 9

LED_GREEN = 10 
LED_RED = 22

redThresholdArea = 10000
greenThresholdArea = 10000

#SERVO_PIN = 4
#FREQ = 50

def gpioInit():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LED_GREEN, GPIO.OUT)  
	GPIO.setup(LED_RED, GPIO.OUT) 
	GPIO.setup(SWITCH_CAPTURE, GPIO.IN, pull_up_down = GPIO.PUD_UP)  
	GPIO.setup(SWITCH_STOP, GPIO.IN, pull_up_down = GPIO.PUD_UP) 
	GPIO.output(LED_RED, True)
	GPIO.output(LED_GREEN, True)
	#GPIO.setup(SERVO_PIN, GPIO.OUT)
	

# Class for LCD
class LCD:

	def __init__(self, lcd_rs,lcd_en,lcd_d4,lcd_d5,lcd_d6,lcd_d7):

		self.LCD_RS = lcd_rs
		self.LCD_EN = lcd_en
		self.LCD_D4 = lcd_d4
		self.LCD_D5 = lcd_d5
		self.LCD_D6 = lcd_d6
		self.LCD_D7 = lcd_d7

		self.LCD_WIDTH = 20
		self.LCD_CHR = True
		self.LCD_CMD = False

		self.LCD_LINE_1 = 0x80
		self.LCD_LINE_2 = 0xC0

		self.E_PULSE = 0.0001
		self.E_DELAY = 0.0001

		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.LCD_EN, GPIO.OUT)  # E
		GPIO.setup(self.LCD_RS, GPIO.OUT) # RS
		GPIO.setup(self.LCD_D4, GPIO.OUT) # DB4
		GPIO.setup(self.LCD_D5, GPIO.OUT) # DB5
		GPIO.setup(self.LCD_D6, GPIO.OUT) # DB6
		GPIO.setup(self.LCD_D7, GPIO.OUT) # DB7

		self.lcd_init()

		# Initialise display
		self.lcd_init()

		# Send some test
		#self.lcd_byte(0x80, False)
		#self.lcd_string("Rasbperry Pi")
		#self.lcd_byte(0xC0, False)
		#self.lcd_string("Model B")
		#time.sleep(5)

	def lcd_init(self):
		# Initialise display
		self.lcd_byte(0x33,False)
		self.lcd_byte(0x32,False)
		self.lcd_byte(0x28,False)
		self.lcd_byte(0x0C,False)
		self.lcd_byte(0x06,False)
		self.lcd_byte(0x01,False)  

	def lcd_string(self, message):
		# Send string to display

		message = message.ljust(self.LCD_WIDTH," ") 

		for i in range(self.LCD_WIDTH):
			self.lcd_byte(ord(message[i]),True)

	def lcd_byte(self,bits, mode):
		# Send byte to data pins
		# bits = data
		# mode = True  for character
		#        False for command

		GPIO.output(self.LCD_RS, mode) # RS

		# High bits
		GPIO.output(self.LCD_D4, False)
		GPIO.output(self.LCD_D5, False)
		GPIO.output(self.LCD_D6, False)
		GPIO.output(self.LCD_D7, False)
		if bits&0x10==0x10:
			GPIO.output(self.LCD_D4, True)
		if bits&0x20==0x20:
			GPIO.output(self.LCD_D5, True)
		if bits&0x40==0x40:
			GPIO.output(self.LCD_D6, True)
		if bits&0x80==0x80:
			GPIO.output(self.LCD_D7, True)

		# Toggle 'Enable' pin
		time.sleep(self.E_DELAY)
		GPIO.output(self.LCD_EN, True)
		time.sleep(self.E_PULSE)
		GPIO.output(self.LCD_EN, False)
		time.sleep(self.E_DELAY)     

		# Low bits
		GPIO.output(self.LCD_D4, False)
		GPIO.output(self.LCD_D5, False)
		GPIO.output(self.LCD_D6, False)
		GPIO.output(self.LCD_D7, False)
		if bits&0x01==0x01:
			GPIO.output(self.LCD_D4, True)
		if bits&0x02==0x02:
			GPIO.output(self.LCD_D5, True)
		if bits&0x04==0x04:
			GPIO.output(self.LCD_D6, True)
		if bits&0x08==0x08:
			GPIO.output(self.LCD_D7, True)

		# Toggle 'Enable' pin
		time.sleep(self.E_DELAY)
		GPIO.output(self.LCD_EN, True)
		time.sleep(self.E_PULSE)
		GPIO.output(self.LCD_EN, False)
		time.sleep(self.E_DELAY)   

# Class for Image Processing
class Image:

	def __init__(self, _camera, _interval, filt, rlowHSV, rhighHSV, glowHSV, ghighHSV):

		# store 
		self.cameraIndex = _camera
		self.frameInterval = _interval
		self.filt = filt
		self.capture = cv.VideoCapture(self.cameraIndex)

		#self.fwindow = cv.namedWindow("Result")
		self.redlowHSV = rlowHSV
		self.redhighHSV = rhighHSV

		self.greenlowHSV = glowHSV
		self.greenhighHSV = ghighHSV

		self.greenlastX = 0
		self.greenlastY = 0
		self.redlastX 	= 0
		self.redlastY 	= 0

		self.red_count = 0
		self.green_count = 0

		self.green_flag = False
		self.red_flag = False

	def clearFlags(self):
		self.red_flag = False
		self.green_flag = False

	def clearCounts(self):
		self.red_count = 0
		self.green_count = 0

	def getFlags(self):
		if self.red_flag == True and self.green_flag == True:
			return -1 # Both found
		elif self.red_flag == False and self.green_flag == False:
			return -2 # Not found
		elif self.red_flag == True and self.green_flag == False:
			return 1 # Success
		elif self.red_flag == False and self.green_flag == True:
			return 2

	def doCaptureAndProcess(self):
		#_frame = ''
		for i in range(10):
			_, _frame = self.capture.read()

		frame = cv.resize(_frame, (640, 480)) 

		if self.filt == 0:
			filteredImage = frame
		elif self.filt == 1:
			filteredImage = cv.blur(frame, (5,5))
		elif self.filt == 2:
			filteredImage = cv.GaussianBlur(frame, (5,5),0)
		elif self.filt == 3:
			filteredImage = cv.medianBlur(frame, 5)
		elif self.filt == 4:
			filteredImage = cv.bilateralFilter(frame,9,75,75)

		kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(5,5))

		hsvImage = cv.cvtColor(filteredImage, cv.COLOR_BGR2HSV)

		RedimgThresholded = cv.inRange(hsvImage, self.redlowHSV, self.redhighHSV)
		GreenimgThresholded = cv.inRange(hsvImage, self.greenlowHSV, self.greenhighHSV)
		erosion = cv.erode(RedimgThresholded,kernel,iterations = 1)
		dilation = cv.dilate(RedimgThresholded,kernel,iterations = 1)

		dilation = cv.dilate(RedimgThresholded,kernel,iterations = 1)
		erosion = cv.erode(RedimgThresholded,kernel,iterations = 1)

		erosion = cv.erode(GreenimgThresholded,kernel,iterations = 1)
		dilation = cv.dilate(GreenimgThresholded,kernel,iterations = 1)

		dilation = cv.dilate(GreenimgThresholded,kernel,iterations = 1)
		erosion = cv.erode(GreenimgThresholded,kernel,iterations = 1)

		Redmoments = cv.moments(RedimgThresholded)
		Greenmoments = cv.moments(GreenimgThresholded)

		Redm01  = Redmoments['m01']
		Redm10  = Redmoments['m10']
		Redarea = Redmoments['m00']

		Greenm01  = Greenmoments['m01']
		Greenm10  = Greenmoments['m10']
		Greenarea = Greenmoments['m00']

		if Redarea > redThresholdArea:
			#print "Red"
			posX = Redm10 / Redarea
			posY = Redm01 / Redarea

			if self.redlastX >= 0 and self.redlastY >= 0 and posX >= 0  and posY >= 0:
				cv.line(filteredImage, (int(posX), int(posY)), (int(self.redlastX),int(self.redlastY)), (0,0,255), 5)

			redlastX = posX
			redlastY = posY
			if Redarea > Greenarea:
				if gpioEnable == True:
					GPIO.output(LED_GREEN, False)
					GPIO.output(LED_RED, True)
				self.red_count += 1
				self.red_flag = True

		if Greenarea > greenThresholdArea:
			posX = Greenm10 / Greenarea
			posY = Greenm01 / Greenarea

			if self.greenlastX >= 0 and self.greenlastY >= 0 and posX >= 0  and posY >= 0:
				cv.line(filteredImage, (int(posX), int(posY)), (int(self.greenlastX),int(self.greenlastY)), (0,0,255), 5)

			greenlastX = posX
			greenlastY = posY

			if Greenarea > Redarea:
				if gpioEnable == True:
					GPIO.output(LED_GREEN, True)
					GPIO.output(LED_RED, False)
				self.green_count += 1
				self.green_flag = True

		# Calculate percentage area
		red_per = (Redarea * 100) / (640 * 480)
		green_per = (Greenarea * 100) / (640 * 480)

		print "red: " + str("%.2f" % red_per) + "%"
		print "green: " + str("%.2f" % green_per) + "%"

		if lcdEnable == True:
			lcd.lcd_byte(0xC0, False)
			lcd.lcd_string(str(self.green_count))
			lcd.lcd_string(", ")
			lcd.lcd_string(str(self.red_count))
			lcd.lcd_string("        ")
			lcd.lcd_byte(0x94, False)
			lcd.lcd_string("red: " + str("%.2f" % red_per) + "%")
			lcd.lcd_byte(0xD4, False) 
			lcd.lcd_string("green: " + str("%.2f" % green_per) + "%")

		cv.imshow("Red Thresholded Image", RedimgThresholded)
		cv.imshow("Green Thresholded Image", GreenimgThresholded)
		cv.imshow("Original Image", filteredImage)

redlowHSV = np.array([170,150,60], dtype=np.uint8)
redhighHSV = np.array([179,255,255], dtype=np.uint8)

greenlowHSV = np.array([60,150,60], dtype=np.uint8)
greenhighHSV = np.array([70,255,255], dtype=np.uint8)

#os.system("servod")
time.sleep(1)

obj = Image(0, 1, 1, redlowHSV, redhighHSV, greenlowHSV, greenhighHSV)
if gpioEnable == True:
	gpioInit()

# If LCD is enabled enable lcd for display
if lcdEnable == True:
	lcd = LCD(LCD_RS, LCD_E, LCD_D4, LCD_D5,LCD_D6,LCD_D7)
	time.sleep(2)
	lcd.lcd_byte(0x80, False)
	lcd.lcd_string("  Color Detection  ")

def process():
	count = 0
	while count < 10:
		obj.doCaptureAndProcess()
		count += 1
		res = obj.getFlags()
		if res == -1:
			print "Both Found"
		elif res == -2:
			print "Not Found"
		elif res == 1:
			os.system("echo 0=100% > /dev/servoblaster")
			print "Red"
			count = 11
			time.sleep(5)
			os.system("echo 0=50% > /dev/servoblaster")
		elif res == 2:
			print "Green"
			os.system("echo 0=0% > /dev/servoblaster")
			count = 11
			time.sleep(5)
			os.system("echo 0=50% > /dev/servoblaster")
	obj.clearFlags()

os.system("servod --p1pins=7,0,0,0,0,0,0,0")
#time.sleep(1)
os.system("echo 0=50% > /dev/servoblaster")
print "System started"
obj.doCaptureAndProcess()
#Servo = GPIO.PWM(SERVO_PIN, FREQ)
while True:

	#process()
	if gpioEnable == True:
		if GPIO.input(SWITCH_CAPTURE) == 0:
			print "Capture"
			time.sleep(0.3)
			process()
		if GPIO.input(SWITCH_STOP) == 0:
			print "Clear"
			time.sleep(0.3)
			obj.clearCounts()

	# Break Operation if key interrupt
	k = cv.waitKey(5) & 0xFF
	if k == 27:
		break
