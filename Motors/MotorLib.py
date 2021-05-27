# Python Script
# https://www.electronicshub.org/raspberry-pi-l298n-interface-tutorial-control-dc-motor-l298n-raspberry-pi/

import RPi.GPIO as GPIO
from time import sleep

M1_in1 = 16
M1_in2 = 15
M1_en = 37

M2_in1 = 18
M2_in2 = 22
M2_en = 13
temp1=1

class Motor:
	def __init__(self, in1, in2, en):
		self.in1 = in1
		self.in2 = in2
		self.en = en
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(in1,GPIO.OUT)
		GPIO.setup(in2,GPIO.OUT)
		GPIO.setup(en,GPIO.OUT)
		GPIO.output(in1,GPIO.LOW)
		GPIO.output(in2,GPIO.LOW)
		self.pwm=GPIO.PWM(en,1000)
		self.pwm.start(0)

	def forward(self, speed):
		GPIO.output(self.in1,GPIO.HIGH)
		GPIO.output(self.in2,GPIO.LOW)
		self.pwm.ChangeDutyCycle(speed)

	def backward(self, speed):
		GPIO.output(self.in1,GPIO.LOW)
		GPIO.output(self.in2,GPIO.HIGH)
		self.pwm.ChangeDutyCycle(speed)

	def exit(self):
		GPIO.output(self.in1,GPIO.LOW)
		GPIO.output(self.in2,GPIO.LOW)
		self.pwm.ChangeDutyCycle(0)

class Motors:
	def __init__(self):
		GPIO.setwarnings(False)
		self._motor1 = Motor(M1_in1, M1_in2, M1_en)
		self._motor2 = Motor(M2_in1, M2_in2, M2_en)

	def forward(self, speed):
		self._motor1.forward(speed)
		self._motor2.forward(speed)

	def backward(self, speed):
		self._motor1.backward(speed)
		self._motor2.backward(speed)

	def left(self, speed):
		self._motor1.backward(speed)
		self._motor2.forward(speed)

	def right(self, speed):
		self._motor1.forward(speed)
		self._motor2.backward(speed)

	def exit(self):
		self._motor1.exit()
		self._motor2.exit()


if (__name__ == '__main__'):
	GPIO.setwarnings(False)
	motors = Motors()
	try:
		print("START")
		while (True):
			motors.forward(40)
			sleep(1)
			motors.forward(0)
			sleep(1)
			motors.left(40)
			sleep(1)
			motors.forward(0)
			sleep(1)
			motors.backward(50)
			sleep(1)
			motors.forward(0)
			sleep(1)
			motors.right(50)
			sleep(1)
			motors.forward(0)
			sleep(1)
	except:
		print("FAILED")
	finally:
		motors.exit()
		print("EXIT")
