#! /usr/bin/env python3

import numpy as np
import rospy
import math
import sys

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


class FreeSpace:

    def __init__(self):
        rospy.Subscriber("mouse/scan", LaserScan, self.scan_callback)
        self.pub = rospy.Publisher("mouse/cmd_vel", Twist, queue_size=10)
        rospy.spin()

    def scan_callback(self, msg):
        ranges = msg.ranges
        ranges = np.array(ranges)
        angle_min = msg.angle_min  # start angle of the scan [rad]
        angle_max = msg.angle_max  # end angle of the scan [rad]
        angle_increment = msg.angle_increment  # angular distance between measurements [rad]
        angles = np.arange(angle_min, angle_max + angle_increment, angle_increment)

        angles = angles[np.isfinite(ranges)]  # delete 'inf'
        ranges = ranges[np.isfinite(ranges)]

        n = 50
        ids = np.arange(len(ranges)) // n
        ranges_average = np.bincount(ids, ranges) / np.bincount(ids)
        angles_average = np.bincount(ids, angles) / np.bincount(ids)
        i = np.argmax(ranges_average)
        freespace_range = ranges_average[i]
        freespace_angle = angles_average[i]

        F_y = np.sin(freespace_angle) * freespace_range
        omega = F_y * 1.0  # scaling force to rotational speed

        output = Twist()  # command velocity is a twist message
        output.linear.x = 0.22  # 0.22 m/s is the max. linear velocity of our turtlebot
        output.angular.z = omega
        self.pub.publish(output)


if __name__ == "__main__":
    rospy.init_node("free_space")
    try:
        node = FreeSpace()
    except rospy.ROSInterruptException:
        pass
