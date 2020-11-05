#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

class TwistToMotors():
    def __init__(self):
        rospy.init_node("twist_to_motors")
        self.pub_lmotor = rospy.Publisher('/speed_left', Int16, queue_size=10)
        self.pub_rmotor = rospy.Publisher('/speed_right', Int16, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.twistCallback)

        self.left = 0
        self.right = 0
        self.dx = 0
        self.dr = 0

    def spin(self):
        while not rospy.is_shutdown():
            self.spinOnce()
        rospy.spin()

    def spinOnce(self):
        self.right = (self.dx + 0.15 * self.dr / 2) / 0.065
        self.left = (self.dx - 0.15 * self.dr / 2) / 0.065

        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)

    def twistCallback(self,msg):
        self.dx = msg.linear.x
        self.dr = msg.angular.z

if __name__ == '__main__':
    twistToMotors = TwistToMotors()
    twistToMotors.spin()
