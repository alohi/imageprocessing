
import cv2 as cv
import numpy as np
import time
import sys

lcdEnable = False

if lcdEnable == True:
	import RPi.GPIO as GPIO

# Define GPIO to LCD mapping
LCD_RS = 7
LCD_E  = 8
LCD_D4 = 25
LCD_D5 = 24
LCD_D6 = 23
LCD_D7 = 18

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

		self.E_PULSE = 0.00005
		self.E_DELAY = 0.00005

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
		self.lcd_byte(LCD_LINE_1, LCD_CMD)
		self.lcd_string("Rasbperry Pi")
		self.lcd_byte(LCD_LINE_2, LCD_CMD)
		self.lcd_string("Model B")

	def lcd_init(self):
		# Initialise display
		self.lcd_byte(0x33,LCD_CMD)
		self.lcd_byte(0x32,LCD_CMD)
		self.lcd_byte(0x28,LCD_CMD)
		self.lcd_byte(0x0C,LCD_CMD)
		self.lcd_byte(0x06,LCD_CMD)
		self.lcd_byte(0x01,LCD_CMD)  

	def lcd_string(message):
		# Send string to display

		message = message.ljust(LCD_WIDTH," ") 

		for i in range(LCD_WIDTH):
			self.lcd_byte(ord(message[i]),LCD_CHR)

	def lcd_byte(bits, mode):
		# Send byte to data pins
		# bits = data
		# mode = True  for character
		#        False for command

		GPIO.output(LCD_RS, mode) # RS

		# High bits
	h	GPIO.output(LCD_D4, False)
		GPIO.output(LCD_D5, False)
		GPIO.output(LCD_D6, False)
		GPIO.output(LCD_D7, False)
		if bits&0x10==0x10:
			GPIO.output(LCD_D4, True)
		if bits&0x20==0x20:
			GPIO.output(LCD_D5, True)
		if bits&0x40==0x40:
			GPIO.output(LCD_D6, True)
		if bits&0x80==0x80:
			GPIO.output(LCD_D7, True)

		# Toggle 'Enable' pin
		time.sleep(E_DELAY)
		GPIO.output(LCD_EN, True)
		time.sleep(E_PULSE)
		GPIO.output(LCD_EN, False)
		time.sleep(E_DELAY)     

		# Low bits
		GPIO.output(LCD_D4, False)
		GPIO.output(LCD_D5, False)
		GPIO.output(LCD_D6, False)
		GPIO.output(LCD_D7, False)
		if bits&0x01==0x01:
			GPIO.output(LCD_D4, True)
		if bits&0x02==0x02:
			GPIO.output(LCD_D5, True)
		if bits&0x04==0x04:
			GPIO.output(LCD_D6, True)
		if bits&0x08==0x08:
			GPIO.output(LCD_D7, True)

		# Toggle 'Enable' pin
		time.sleep(E_DELAY)
		GPIO.output(LCD_EN, True)
		time.sleep(E_PULSE)
		GPIO.output(LCD_EN, False)
		time.sleep(E_DELAY)   

# Class for Image Processing
class Image:

	def __init__(self, _camera, _interval, filt, lowHSV, highHSV):

		# store 
		self.cameraIndex = _camera
		self.frameInterval = _interval
		self.filt = filt
		self.capture = cv.VideoCapture(self.cameraIndex)

		self.fwindow = cv.namedWindow("Result")
		self.lowHSV = lowHSV
		self.highHSV = highHSV

	def doCaptureAndProcess(self):
		_, frame = self.capture.read()

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

		hsvImage = cv.cvtColor(filteredImage, cv.COLOR_BGR2HSV)
		maskedImage = cv.inRange(hsvImage, self.lowHSV, self.highHSV)
		result = cv.bitwise_and(filteredImage,filteredImage,mask = maskedImage)

		cv.imshow("Result", result)

###########################################
## Main Code
if len(sys.argv) != 10:
	print "Error"
	sys.exit(0)

# Parse args
cam = int(sys.argv[1])
interval = int(sys.argv[2])
highh = int(sys.argv[3])
highs = int(sys.argv[4])
highv = int(sys.argv[5])
lowh = int(sys.argv[6])
lows = int(sys.argv[7])
lowv = int(sys.argv[8])
filt = int(sys.argv[9])

# Print Value
print "Camera Index : " + str(cam)
print "Interval : " + str(interval)
print "HIGH H : " + str(highh)
print "HIGH S : " + str(highs)
print "HIGH V : " + str(highv)
print "LOW H : " + str(lowh)
print "LOW S : " + str(lows)
print "LOW V : " + str(lowv)
print "FILTER : " + str(filt)

# Construct an array for inRange filter
lowHSV = np.array([lowh,lows,lowv], dtype=np.uint8)
highHSV = np.array([highh,highs,highv], dtype=np.uint8)

obj = Image(cam, interval, filt, lowHSV, highHSV)

# If LCD is enabled enable lcd for display
if lcdEnable == True:
	lcd = LCD(LCD_RS, LCD_E, LCD_D4, LCD_D5,LCD_D6,LCD_D7)

while True:

	obj.doCaptureAndProcess()

	# Break Operation if key interrupt
	k = cv.waitKey(5) & 0xFF
	if k == 27:
		break