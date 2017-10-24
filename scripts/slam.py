#!/usr/bin/env python

## Localization and Mapping node

import rospy
from gemoetry_msgs.msg import Pose2D
from fiducial_msgs.msg import FiducialTransformArray

pub = rospy.Publisher('map_pose', Pose2D)
def callback(tf_msg):

def main():




if __name__ == '__main__':
    rospy.init_node('slam_node')
    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, callback)
    rospy.spin()
    main()
