#! /usr/bin/env python3
# license removed for brevity

import rospy
import numpy as np
import rogata_library.rogata_library as rgt

# from scripts.cheese import Cheese
from nav_msgs.msg import Odometry
from scripts.move_to_cheese import get_cheese_positions

rgt_helper = rgt.rogata_helper()


class Mouse:

    def __init__(self):
        # x,y pos of target cheese and mouse
        self.target_cheese = 0
        self.position = 0

        # callback for mouse position
        rospy.Subscriber("mouse_obj/odom", Odometry, self.mouse_callback)

        self.cheese_array = get_cheese_positions("MAP_2")

        rospy.spin()

    def cheese_callback(self, data):
        # evaluate all cheese
        for cheese in self.cheese_array:
            cheese.evaluate(self.position, self.cat.position)

        # sort array by score
        self.cheese_array(key=lambda x: x.score, reverse=True)

        # get coordinates of cheese with best evaluation score
        x, y = self.cheese_array[0].position

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
