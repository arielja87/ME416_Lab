#!/usr/bin/env python

''' ROS node: integrator_node
	Subscribes to raw imu data, integrates those data and publishes a 2D position
'''

import rospy
from numpy import mean
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D



pub_p = rospy.Publisher('pose', Pose2D, queue_size=1)

def integrate(ft, Ft, dt): # f(t) represents the stream of t-values to be integrated F(t) is the previous value of the integral and dt is the time duration between samples
    #--------- stub here ? -----------------#
    Ft = Ft + dt*ft
    return(Ft)

def parse_msg_and_publish(imu_msg):


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
