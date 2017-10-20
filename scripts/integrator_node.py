#!/usr/bin/env python

''' ROS node: integrator_node
	Subscribes to raw imu data, integrates those data and publishes a 2D position
'''

import rospy
from lab_module import combine
from numpy import mean
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D

# Initial conditions for integration
class initial_conditions:
    def __init__(self):
	self.v_x = 0
    	self.v_y = 0
    	self.x = 0
    	self.y = 0
    	self.theta = 0

pub_p = rospy.Publisher('pose', Pose2D, queue_size=1)
pub_v = rospy.Publisher('velocity', Pose2D, queue_size=1)
pose_msg = Pose2D()
vel_msg = Pose2D()
t1 = 0
k = 0 # for time step
imu_cal = [] # list to hold imu readings to use to find an average offset
Ft = initial_conditions() # stores integrals
offsets = []
n_samples = 400 # number of initial imu samples to calibrate over

def integrate(ft, Ft, dt): # f(t) represents the stream of t-values to be integrated F(t) is the previous value of the integral and dt is the time duration between samples
    #--------- stub here ? -----------------#
    Ft = Ft + dt*ft
    return(Ft)

def parse_msg_and_publish(imu_msg):
    if not rospy.is_shutdown():
        global t1, k, imu_cal, Fx, offsets, n_samples
        k += 1
        # linear acceleration vector (as a list)
        imu = [imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.angular_velocity.z]
        # calibrate and set offsets
        if (k < n_samples):
            rospy.loginfo_throttle(10, "Calibrating IMU offsets...")
            imu_cal.append(imu)
    	    return
        if (k == n_samples):
            offsets = list(mean(imu_cal, axis=0))
            rospy.loginfo("IMU offsets set to x: %f, y: %f, theta: %f", offsets[0], offsets[1], offsets[2])

        # time of reading
        t2 = imu_msg.header.stamp.secs + imu_msg.header.stamp.nsecs*10**(-9)

        if (t1 == 0]):
            t1 = t2
			return
        else:
            # duration between readings
            dt = t2 - t1
            t1 = t2
            #--------- stub here ? -----------------#
            Ft.v_x = integrate(imu[0] - offsets[0], Ft.v_x, dt)
            Ft.v_y = integrate(imu[1] - offsets[1], Ft.v_y, dt)
            Ft.x = integrate(Ft.v_x, Ft.x, dt)
            Ft.y = integrate(Ft.v_y, Ft.y, dt)
            Ft.theta = integrate(imu[2] - offsets[2], Ft.theta, dt)

        pose_msg.x = Ft.x
        pose_msg.y = Ft.y
        pose_msg.theta = Ft.theta
        pub_p.publish(pose_msg)

        vel_msg.x = Ft.v_x
        vel_msg.y = Ft.v_y
        vel_msg.theta = imu[2] - offsets[2]
        pub_v.publish(vel_msg)

def main():
    rospy.init_node('integrator_node')
    rospy.loginfo("Started: " + rospy.get_name())
    rospy.loginfo("Subscribing to: /rtimulib_node/imu")
    rospy.loginfo("Publishing to: /pose")
    rospy.loginfo("Publishing to: /velocity")
    rospy.Subscriber("rtimulib_node/imu", Imu, callback=parse_msg_and_publish, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
