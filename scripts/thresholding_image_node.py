#!/usr/bin/env python

''' ROS node: hue_threshold_node
	Subscribes to a CompressedImage, performs a binary segmentation of the image based on hue threshold
	and publishes the new raw image and the segmentation centeroid
'''
import rospy, time
import cv2
import numpy as np
import warnings
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge, CvBridgeError

img_pub = rospy.Publisher('segmented_image', Image, queue_size=1)
center_pub = rospy.Publisher('segmented_center', Pose2D, queue_size=1)
bridge = CvBridge()

# bounds for the hsv mask
hsv_lower = np.array([34, 80, 100])
hsv_upper = np.array([80, 255, 255])
seg_time_buff = []
last_center = [0,0]

def segment(img_msg):
    global seg_time_buff, last_center
    t1 = time.time()
    np_img = np.fromstring(img_msg.data, dtype=np.uint8)
    new_img = cv2.cvtColor(cv2.imdecode(np_img, 1), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(new_img, hsv_lower, hsv_upper)
    new_img = np.stack((mask, mask, mask), axis=2)

    with warnings.catch_warnings():
        warnings.filterwarnings('error')
        try:
            # Draw a vertical line through center of segmentation and catch warning if no segmentation found
            center = np.mean(np.nonzero(mask.transpose()), axis=1).astype('uint32')
            cv2.line(new_img, (center[0], 0), (center[0], mask.shape[0]), (0,255,0), 3)
        except Warning:
            if (last_center[0] > (mask.shape[1]/2)):
                center = [mask.shape[1] + 1, mask.shape[0]/2]
            else:
                center = [-1, mask.shape[0]/2]

    t2 = time.time()
    if (seg_time_buff is not None and len(seg_time_buff) < 100):
        seg_time_buff.append(t2-t1)
    if (seg_time_buff is not None and len(seg_time_buff) == 100):
        rospy.loginfo_throttle(5, "segmentation: %.4f" % np.mean(seg_time_buff))
        seg_time_buff = None
    try:
        img_pub.publish(bridge.cv2_to_imgmsg(new_img, "bgr8"))
    except CvBridgeError as e:
        print(e)
    last_center = center
    msg = Pose2D()
    msg.x = float(center[0])
    msg.y = float(center[1])
    center_pub.publish(msg)

def main():
    rospy.init_node('segmented_image_node')
    rospy.loginfo("Started: " + rospy.get_name())
    rospy.loginfo("Subscribing to: /raspicam_node/image/compressed")
    rospy.loginfo("Publishing to: /segmented_image")
    rospy.loginfo("Publishing to: /segmented_center")
    rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, callback=segment, queue_size=1, buff_size=2**18)

if __name__ == '__main__':
    main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


