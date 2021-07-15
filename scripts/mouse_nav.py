#! /usr/bin/env python3
# license removed for brevity

import rospy
import numpy as np
import rogata_library.rogata_library as rgt

from nav_msgs.msg import Odometry

rgt_helper = rgt.rogata_helper()


class Mouse:

    def __init__(self):
        # x,y pos of target cheese and mouse
        self.target_cheese = 0
        self.position = 0

        # callback for mouse position
        rospy.Subscriber("mouse_obj/odom", Odometry, self.mouse_callback)

        # get cheese position
        # TODO: doesnt work!!
        cheese_pos = rgt_helper.available_objects
        print(f"av objs: {cheese_pos}")
        # TODO: use rgt_helper.get_dist() to determine nearest cheese

        rospy.spin()

    def cheese_callback(self, data):
        # TODO: get ie. nearest cheese position
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.target_cheese = np.array([x, y])
        print(f"target cheese: {self.target_cheese}")

    def mouse_callback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.position = np.array([x, y])
        print(f"target mouse pos: {self.position}")


if __name__ == "__main__":
    rospy.init_node("mouse_nav")
    try:
        node = Mouse()
    except rospy.ROSInterruptException:
        pass
