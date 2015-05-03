
import RPi.GPIO as GPIO
import time

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

		self.E_PULSE = 0.001
		self.E_DELAY = 0.001

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

		#message = message.ljust(self.LCD_WIDTH," ") 

		for i in range(len(message)):
			self.lcd_byte(ord(message[i]),True)

	def lcd_byte(self,bits, mode):

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
