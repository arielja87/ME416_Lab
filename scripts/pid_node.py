#!/usr/bin/env python

''' ROS node: pid_node
	Subscribes to a number msg, (float or int), publishes float msgs to the \motor_vel topic
'''

import rospy
import numpy as np
#from lab_module import combine
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose2D, Twist
#from pid_param.cfg import PID_Config

gains_angular = [-1./1410., 0, -1./3440.]

t1 = 0

rospy.init_node('pid_node', anonymous=True)
pub = rospy.Publisher('motor_vel', Twist, queue_size=1)

class reference_error:
    def __init__(self, reference): # Initializes with the desired reference value
     	self.current = 0 # latest error value
	self.previous = 0 # stores the previous error value (for differentiaion)
	self.integral = 0 # stores the integral (the accumulated error)
	self.reference = reference
    def update(self, val):
        self.previous = self.current
        self.current = val - self.reference

class pid_controller:
    def __init__(self, gains):
	self.kp = gains[0]
	self.ki = gains[1]
	self.kd = gains[2]
    def p_term(self, e): # proportional term
	return self.kp * e.current
    def i_term(self, e, dt): # integrator
	e.integral = e.integral + e.previous * dt
	return self.ki * e.integral
    def d_term(self, e, dt): # differentiator
	return self.kd * ((e.current - e.previous) / dt)
    def control(self, e, dt):
	return self.p_term(e) + self.i_term(e, dt) + self.d_term(e, dt)


def dynamic_reconfigure_callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\
        {str_param}, {bool_param}, {size}""".format(**config))
    return config

def control(ctrl_msg):
    if not rospy.is_shutdown():
        global t1
        secs = rospy.get_rostime().secs
        nsecs = rospy.get_rostime().nsecs
        t2 = secs + nsecs*10**(-9)

        if (t1 == 0):
            t1 = t2
        else:
            e.update(ctrl_msg.x)
            # duration between readings
            dt = t2 - t1
            t1 = t2
            #--------- stub here ? -----------------#
            pid_out = pid_angular.control(e, dt)
#            print(pid_out)
            msg = Twist()
            msg.linear.x = .22
            msg.angular.z = pid_out
            pub.publish(msg)

def main():
    rospy.loginfo("Started: " + rospy.get_name())
    rospy.loginfo("Subscribing to: /segmented_center")
    rospy.loginfo("Publishing to: " + rospy.get_name())
    rospy.Subscriber("segmented_center", Pose2D, callback=control, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    pid_angular = pid_controller(gains_angular)
    pid_linear = pid_controller(gains_linear)
    e = reference_error(205.5)
    main()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
