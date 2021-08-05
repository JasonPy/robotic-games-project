#! /usr/bin/env python3

import numpy as np
import rospy
import math
import sys

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


def linear_spring(ranges, angles, max_distance, min_distance, spring_constant):
    reduced_ranges = np.clip(ranges, min_distance, max_distance)  # clip to maximum range (circle)
    delta_r = max_distance - reduced_ranges
    F_0 = delta_r * spring_constant  # linear spring
    force_angles = angles - math.pi  # opposite force direction
    F_x = np.cos(force_angles) * F_0
    F_y = np.sin(force_angles) * F_0
    F_x = np.sum(F_x)
    F_y = np.sum(F_y)
    return np.array([F_x, F_y])


class CollisionAvoidance:

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

        # use ellipse for collisison avoidance
        a = 2  # radius on the x-axis
        b = 0.75  # radius on the y-axis

        # https://en.wikipedia.org/wiki/Ellipse#Polar_form_relative_to_center
        ellipse_ranges = a * b / np.sqrt(
            np.square(b * np.cos(angles)) + np.square(a * np.sin(angles)))  # ellipse ranges are max ranges
        reduced_ranges = np.clip(ranges, 0.1, ellipse_ranges)
        F = linear_spring(ranges, angles, ellipse_ranges, min_distance=0.1, spring_constant=0.02)
        v = 0.22  # v_max
        omega = F[1] * 3.0  # scaling force to rotational speed

        output = Twist()
        output.linear.x = 0.22  # 0.22 m/s is the max. linear velocity of our turtlebot
        output.angular.z = omega
        self.pub.publish(output)


if __name__ == "__main__":
    rospy.init_node("collision_avoidance")
    try:
        node = CollisionAvoidance()
    except rospy.ROSInterruptException:
        pass
