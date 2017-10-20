#!/usr/bin/env python
#
## Simple script that publishes geometry_msgs/Twist messages
## to the '/motor_vel' topic

import rospy
import lab_module
from geometry_msgs.msg import Twist

class key_msg(object):
    def __init__(self, lin, ang):
        self.linear = lin
        self.angular = ang

step = .3

def key_op():
    pub = rospy.Publisher('motor_vel', Twist, queue_size=10)
    rospy.init_node('key_op')
    rate = rospy.Rate(50)
    msg = key_msg(0, 0)
    twist_msg = Twist()
    getch = lab_module._Getch()
    rospy.loginfo("Started: /key_op_node")
    rospy.loginfo("Publishing to: /motor_vel")
    rospy.sleep(1.)
    print("\n\n\n\n\n     Use 'I' and 'K' for linear velocity.")
    print("     Use 'J' and 'L' for angular velocity")
    print("     Press 'Q' to quit.\n\n\n")
    print("\r                      Linear velocity: 0.00, Angular Velocity: 0.00\r"),
    while not rospy.is_shutdown():
        key = getch()
        if key in "lkjiq ":
            if (key == 'i'):
                msg.linear = min(step, msg.linear + step)
            elif (key == 'k'):
                msg.linear = max(-step, msg.linear - step)
            elif (key == 'j'):
                msg.angular = min(step -.1, msg.angular + step - .1)
            elif (key == 'l'):
                msg.angular = max(-(step -.1), msg.angular - (step - .1))
            elif (key == ' '):
                msg.angular = 0
                msg.linear = 0
            print("\r                      Linear velocity: %.2f, Angular Velocity: %.2f " % (msg.linear, msg.angular)),
            twist_msg.linear.x = msg.linear
            twist_msg.angular.z = msg.angular
            pub.publish(twist_msg)

            if (key == 'q'):
                print("")
                rospy.loginfo("Thanks for playing!")
                rospy.signal_shutdown("Cool beans, that was fun!")
        rate.sleep()

if __name__ == '__main__':
    try:
        key_op()
    except rospy.ROSInterruptException:
        pass
