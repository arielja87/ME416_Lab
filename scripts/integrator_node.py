#!/usr/bin/env python

''' ROS node: integrator_node
	Subscribes to raw imu data, integrates those data and publishes a 2D position
'''

import rospy
from numpy import mean
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D
from math import floor, log10, pi

# Initial conditions for integration
class antiderivatives(object): # antiderivatives are sometimes indicated by a capital F(x)
    def __init__(self):
	self.v_x = 0
    	self.v_y = 0
    	self.x = 0
    	self.y = 0
    	self.theta = 0

pub = rospy.Publisher('pose', Pose2D, queue_size=200)
pose_msg = Pose2D()
t_buff = [0,0]
k = 0 # for time step
imu_cal = [] # list to hold imu readings to use to find an average offset
Fx = antiderivatives()
offsets = []

def integrate(fx, Fx, dt): # f(x) represents the stream of x-values to be integrated
    #--------- stub here ? -----------------#
    Fx = Fx + dt*fx
    return(Fx)

# to combine seconds and nanoseconds in imu_msg/header
def combine(a, b):
    if b == 0:
        return a
    return a + b * 10**-(floor(log10(b))+1)

def parse_msg_and_publish(imu_msg):
    global k, imu_cal, Fx, offsets
    k += 1
    # linear acceleration vector (as a list)
    imu = [imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.angular_velocity.z]
    # calibrate and set offsets
    if (k < 200):
        rospy.loginfo_throttle(10, "Calibrating imu offsets...")
        imu_cal.append(imu)
	return
    if (k == 200):
        offsets = list(mean(imu_cal, axis=0))
        print (offsets)
	rospy.loginfo("Imu offsets set to x: %f, y: %f, theta: %f", offsets[0], offsets[1], offsets[2])

    # time of reading
    t = combine(imu_msg.header.stamp.secs, imu_msg.header.stamp.nsecs)

    if (t_buff == [0,0]):
        t_buff.pop(0)
        t_buff.append(t)
    else:
        t_buff.pop(0)
        t_buff.append(t)
        # duration between readings
        dt = t_buff[1] - t_buff[0]
        #--------- stub here ? -----------------#
        Fx.v_x = integrate(imu[0] - offsets[0], Fx.v_x, dt)
        Fx.v_y = integrate(imu[1] - offsets[1], Fx.v_y, dt)
        Fx.x = integrate(Fx.v_x, Fx.x, dt)
        Fx.y = integrate(Fx.v_y, Fx.y, dt)
        Fx.theta = integrate(imu[2] - offsets[2], Fx.theta, dt)

    pose_msg.x = Fx.x
    pose_msg.y = Fx.y
    pose_msg.theta = Fx.theta
    pub.publish(pose_msg)

def integrator_node():
    rospy.init_node('integrator_node')
    rospy.loginfo("Started: " + rospy.get_name())
    rospy.loginfo("Subscribing to: /rtimulib_node/imu")
    rospy.loginfo("Publishing to: /pose")
    rospy.Subscriber("rtimulib_node/imu", Imu, callback=parse_msg_and_publish)
    rospy.spin()

if __name__ == '__main__':
    try:
        integrator_node()
    except rospy.ROSInterruptException:
        pass
