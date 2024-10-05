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
        self.v_left = 0.0
        self.v_right = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.rate = rospy.Rate(50)
    
    def callback(self, msg):
        self.vx = msg.vector.x
        self.vy = msg.vector.y
    
    def run(self):
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            self.vx = (self.v_left + self.v_right) / 2
            self.vth = (self.v_right - self.v_left) / 0.32
            dt = (self.current_time - self.last_time).to_sec()
            delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
            delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
            delta_th = self.vth * dt
            self.x += delta_x
            self.y += delta_y
            self.th += delta_th
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*quaternion_from_euler(0, 0, self.th)))
            odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
            self.last_time = self.current_time
            self.pub.publish(odom)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rawOdom = RawOdom()
        rawOdom.run()
    except rospy.ROSInterruptException:
        pass
