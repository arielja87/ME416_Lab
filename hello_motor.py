import sys
import time
import RPi.GPIO as GPIO

# Set the pin numbering scheme to board numbers (as opposed to Broadcom number, see the pinout)
GPIO.setmode(GPIO.BOARD)

# Assign variable names to the pins so you don't have to control them by number
L_forward_pin = 31
L_backward_pin = 29
L_speed_pin = 33
R_forward_pin = 8
R_backward_pin = 10
R_speed_pin = 12

# Assign some useful parameters, in this case, max_speed is the duty cycle of the pwm signal (as a percentage) that will
# control the speed of the motors. It is good practice to keep the duty cycle at or below 90%, so we should set speeds 
# to be some fraction of this number (cast as an integer, see below).
max_speed = 90

# Pin setup. In this test case, we're only performing open loop control, so all the motor pins (our actuators) are set to output.
GPIO.setup(L_forward_pin, GPIO.OUT)
GPIO.setup(R_forward_pin, GPIO.OUT)
GPIO.setup(L_backward_pin, GPIO.OUT)
GPIO.setup(R_backward_pin, GPIO.OUT)
GPIO.setup(L_speed_pin, GPIO.OUT)
GPIO.setup(R_speed_pin, GPIO.OUT)

# By creating pwm objects, we can easily control their duty cycles. There's only 2 pwm channels on the Pi, these are them.
# Sets up a pwm signal with a frequency of 100 Hz. The frequency is analogous to the resolution of the pwm signal. 
# The wheels, chassis, and gear box result in a relatively high inertial load on the motors, which allows us to set a 
# relatively low frequency for the pwm signal. If the motor is jerky, try increasing this by 100 Hz.
L_pwm = GPIO.PWM(L_speed_pin, 100) 
R_pwm = GPIO.PWM(R_speed_pin, 100)

# Some useful functions for robot velocity.
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

# Starts the pwm oscillators at the max_speed duty cycle (90%)
L_pwm.start(max_speed) 
R_pwm.start(max_speed)

# Everything is set up, let's roll
setspeed(int(.5*max_speed), int(.5*max_speed)) # duty cycle percentage cast as an integer.
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
setspeed(int(.5*max_speed), int(.3*max_speed))
forward(1)
time.sleep(1)
print("Arc Left")
setspeed(int(.3*max_speed), int(.5*max_speed))
forward(1)

# This basically resets any used pins to input mode to avoid bad stuff like shorts
GPIO.cleanup()
