#!/usr/bin/env python

import sys
import time
import rospy
from geometry_msgs.msg import Twist
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

# The motors will most likely not spin exactly at the same speed, this is a simple factor to attempt to account for this.
# multiply the faster motor by this offset.
speed_offset = .945


# Assign some useful parameters, in this case, max_duty_cycle is the duty cycle of the pwm signal (as a percentage) that will
# control the speed of the motors. It is good practice to keep the duty cycle at or below 90%, so we should set speeds
# to be some fraction of this number (cast as an integer, see below).
max_duty_cycle = 80

# Pin setup. In this test case, we're only performing open loop control, so all the motor pins (our actuators) are set to output.
GPIO.setup(L_forward_pin, GPIO.OUT)
GPIO.setup(R_forward_pin, GPIO.OUT)
GPIO.setup(L_backward_pin, GPIO.OUT)
GPIO.setup(R_backward_pin, GPIO.OUT)
GPIO.setup(L_speed_pin, GPIO.OUT)
GPIO.setup(R_speed_pin, GPIO.OUT)

# By creating pwm objects, we can easily control their duty cycles. There's only 2 pwm channels on the Pi, these are them.
# Sets up a pwm signal with a frequency of 25 Hz. The frequency is analogous to the resolution of the pwm signal.
# The wheels, chassis, and gear box result in a relatively high inertial load on the motors, which allows us to set a
# relatively low frequency for the pwm signal. If the motor is jerky, try increasing this by 5 Hz.
L_pwm = GPIO.PWM(L_speed_pin, 100)
R_pwm = GPIO.PWM(R_speed_pin, 100)

# Some useful functions for robot velocity.

def setspeed(L_speed, R_speed):
# takes speeds as floats in the interval [0,1]
    L_pwm.ChangeDutyCycle(int(abs(L_speed  * speed_offset) * max_duty_cycle)) # My left motor is faster
    R_pwm.ChangeDutyCycle(int(abs(R_speed) * max_duty_cycle))

    if (L_speed < 0):
        GPIO.output(L_backward_pin, GPIO.HIGH)
        GPIO.output(L_forward_pin, GPIO.LOW)
    elif (L_speed == 0):
        GPIO.output(L_backward_pin, GPIO.LOW)
        GPIO.output(L_forward_pin, GPIO.LOW)
    else:
        GPIO.output(L_backward_pin, GPIO.LOW)
        GPIO.output(L_forward_pin, GPIO.HIGH)

    if (R_speed < 0):
        GPIO.output(R_backward_pin, GPIO.HIGH)
        GPIO.output(R_forward_pin, GPIO.LOW)
    elif (R_speed == 0):
        GPIO.output(R_backward_pin, GPIO.LOW)
        GPIO.output(R_forward_pin, GPIO.LOW)
    else:
        GPIO.output(R_backward_pin, GPIO.LOW)
        GPIO.output(R_forward_pin, GPIO.HIGH)

def translate_twist(twist_msg):
    if not rospy.is_shutdown():
        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z
        #--------- stub here? --------------------#
        L_speed = max(-1, min(-angular_velocity + linear_velocity, 1))
        R_speed = max(-1, min( angular_velocity + linear_velocity, 1))
        setspeed(L_speed, R_speed)

def motors_node():
    rospy.init_node('motors_node')
    rospy.Subscriber('motor_vel', Twist, translate_twist)
    rospy.on_shutdown(GPIO.cleanup)
    rospy.loginfo("Subscribing to topic: /motor_vel")
    # Starts the pwm oscillators at the max_duty_cycle duty cycle (90%)
    L_pwm.start(0)
    R_pwm.start(0)
    rospy.spin()

if __name__ == '__main__':
    motors_node()
