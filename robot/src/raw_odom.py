#!/usr/bin/env python3

import rospy
from math import sin, cos
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3Stamped, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import  quaternion_from_euler


class RawOdom:
    def __init__(self):
        rospy.init_node('raw_odom', anonymous=True)
        self.pub = rospy.Publisher('raw_odom', Odometry, queue_size=10)
        self.sub = rospy.Subscriber('speed', Vector3Stamped, self.callback)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.encLeft = 0
        self.encRight = 0
        self.encLeftOld = 0
        self.encRightOld = 0
        self.DistancePerCount = (3.14159265 * 0.145) / 345.6
        self.dist_left = 0
        self.dist_right = 0
        self.rate = rospy.Rate(20)

    def callback(self, msg):
        self.encLeft = msg.vector.x
        self.encRight = msg.vector.y

    def run(self):
        while not rospy.is_shutdown():
            self.dist_left = (self.encLeft - self.encLeftOld) * self.DistancePerCount
            self.dist_right = (self.encRight - self.encRightOld) * self.DistancePerCount
            self.encLeftOld = self.encLeft
            self.encRightOld = self.encRight

            if self.dist_left == self.dist_right:
                self.x = self.x + self.dist_left * cos(self.theta)
                self.y = self.y + self.dist_left * sin(self.theta)
                w = 0
            else:
                R = (self.dist_left + self.dist_right) / (2 * (self.dist_right - self.dist_left))
                w = (self.dist_right - self.dist_left) / 0.145
                ICCx = self.x - R * sin(self.theta)
                ICCy = self.y + R * cos(self.theta)
                self.x = cos(w) * (self.x - ICCx) - sin(w) * (self.y - ICCy) + ICCx
                self.y = sin(w) * (self.x - ICCx) + cos(w) * (self.y - ICCy) + ICCy
                self.theta = self.theta + w

            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*quaternion_from_euler(0, 0, self.theta)))
            odom.twist.twist = Twist(Vector3((self.dist_left + self.dist_right) / 2, 0, 0), Vector3(0, 0, w))
            self.pub.publish(odom)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rawOdom = RawOdom()
        rawOdom.run()
    except rospy.ROSInterruptException:
        pass
