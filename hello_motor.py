import sys
import time
import RPi.GPIO as GPIO

mode=GPIO.getmode()

GPIO.cleanup()

L_forward_pin = 31
L_backward_pin = 29
L_speed_pin = 33
R_forward_pin = 8
R_backward_pin = 10
R_speed_pin = 12

max_speed = 90
sleeptime = 1

GPIO.setmode(GPIO.BOARD)
GPIO.setup(L_forward_pin, GPIO.OUT)
GPIO.setup(R_forward_pin, GPIO.OUT)
GPIO.setup(L_backward_pin, GPIO.OUT)
GPIO.setup(R_backward_pin, GPIO.OUT)
GPIO.setup(L_speed_pin, GPIO.OUT)
GPIO.setup(R_speed_pin, GPIO.OUT)
L_pwm = GPIO.PWM(L_speed_pin, 1000)
R_pwm = GPIO.PWM(R_speed_pin, 1000)


def setspeed(L_speed, R_speed):
L_pwm.ChangeDutyCycle(L_speed)
R_pwm.ChangeDutyCycle(R_speed)

def forward(x):
GPIO.output(L_forward_pin, GPIO.HIGH)
GPIO.output(R_forward_pin, GPIO.HIGH)
time.sleep(x)
GPIO.output(L_forward_pin, GPIO.LOW)
GPIO.output(R_forward_pin, GPIO.LOW)

def right(x):
GPIO.output(L_forward_pin, GPIO.HIGH)
GPIO.output(R_backward_pin, GPIO.HIGH)
time.sleep(x)
GPIO.output(L_forward_pin, GPIO.LOW)
GPIO.output(R_backward_pin, GPIO.LOW)

def left(x):
GPIO.output(L_backward_pin, GPIO.HIGH)
GPIO.output(R_forward_pin, GPIO.HIGH)
time.sleep(x)
GPIO.output(L_backward_pin, GPIO.LOW)
GPIO.output(R_forward_pin, GPIO.LOW)

def reverse(x):
GPIO.output(L_backward_pin, GPIO.HIGH)
GPIO.output(R_backward_pin, GPIO.HIGH)
time.sleep(x)
GPIO.output(L_backward_pin, GPIO.LOW)
GPIO.output(R_backward_pin, GPIO.LOW)

L_pwm.start(max_speed)
R_pwm.start(max_speed)
setspeed(50, 50)
print("Moving Forward")
forward(1)
time.sleep(1)
print("Turning Right")
right(1)
time.sleep(1)
print("Turning Left")
left(1)
time.sleep(1)
print("Moving Backward")
reverse(1)
time.sleep(1)
print("Arc Right")
setspeed(50, 30)
forward(1)
time.sleep(1)
print("Arc Left")
setspeed(30, 50)
forward(1)

GPIO.cleanup()