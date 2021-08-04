#! /usr/bin/env python3
# license removed for brevity

import rospy
import numpy as np
import copy
import rogata_library.rogata_library as rgt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# TODO: load functions from util file (see issue and comments down below)
# from util import *

rgt_helper = rgt.rogata_helper()


class Mouse:

    def __init__(self):
        # positions of cheese objects (have to be initially loaded since cheese_obj only contains the mean
        # cheese coordinate)
        self.cheese_amount = 4
        self.cheese_positions = np.zeros(shape=(self.cheese_amount, 2))
        # x,y pos of target cheese and mouse
        self.target_cheese = 0
        self.position = 0
        self.speed = 0.2  # have to determine that?

        # callback for mouse and cat position
        rospy.Subscriber("mouse_obj/odom", Odometry, self.odom_mouse_callback)
        rospy.Subscriber("cat_obj/odom", Odometry, self.odom_cat_callback)
        rospy.Subscriber("mouse/cmd_vel", Twist, self.cmd_vel_mouse_callback)  # fake subscriber

        # publisher for mouse velocities
        self.pub = rospy.Publisher("mouse/cmd_vel", Twist, queue_size=10)  # or mouse_cmd_vel?

        print("--------------- game is starting ---------------")

        while not rospy.is_shutdown():

            if self.cat_is_in_near_mouse():
                # if cat enters mouse cell -> flee using minimax-approach
                print("flee from cat")

            else:
                # determine next best move in the 'absence' of the cat -> optimal homing + collision avoidance,
                # then: think of what the cat might be doing (while the cat is also thinking about what we might
                # be doing)
                print("go primarily to the cheese, but keep the cat in mind")
                # homing/navigation/collision avoidance

        # why are we using a loop instead of rospy.spin() -- what is that for?
        # rospy.spin() effectively goes into an infinite loop until it receives a shutdown signal (e.g. ctrl-c).
        # During that loop it will process any events that occur, such as data received on a topic or a timer
        # triggering, when they occur but otherwise it will sleep. You should use spin() when your node doesn't do
        # anything outside of its callbacks. -> we don't need rospy.spin() but the while loop

    # TODO: update the target cheese position (nearest cheese for example)
    def update_target_cheese(self):
        """
        This function reevaluates the cheese the mouse is trying to navigate to and updates the variable target_cheese.
        This function is called whenever the mouse needs to decide its next move.
        """
        # TODO: use rgt_helper.get_dist() to determine nearest cheese
        cheese_scores = np.zeros(self.cheese_amount)

        for j in range(self.cheese_positions.shape[0]):
            # determine the current score for every cheese
            cheese_scores[j] = self.get_cheese_score(self.cheese_positions[j, :])

        # update target_cheese
        self.target_cheese = self.cheese_positions[np.argmax(cheese_scores), :]
        print(f"target cheese: {self.target_cheese}")

    # TODO
    def get_cheese_score(self, cheese_pos):
        """
        This function determines the score of a cheese dependent on certain criteria.
        :param cheese_pos: coordinates of the cheese
        :return: score of the cheese
        """
        score = 0

        return score

    # TODO
    def get_flee_from_cat_score(self):
        """
        Determine how urgent it is too flee from the cat.
        :return: 0-1, 1 = 100% flee, 0 = 0% flee (so go to the cheese)
        """

    # TODO
    def cat_is_in_near_mouse(self):
        """
        This function checks if the cat is inside the mouse cell.
        :return:
        """

        return False

    # TODO: update target_cheese?
    def odom_mouse_callback(self, data):
        orientation = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])[2]
        x_pos = data.pose.pose.position.x
        y_pos = data.pose.pose.position.y

        self.position = np.array([x_pos, y_pos])

        # self.update_target_cheese() ?

    # TODO update target_cheese?
    def odom_cat_callback(self, data):
        """
        This function updates the position of the cat in the game_state.
        """
        orientation = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])[2]
        x_pos = data.pose.pose.position.x
        y_pos = data.pose.pose.position.y

        self.position = np.array([x_pos, y_pos])

        # self.update_target_cheese() ?

    def cmd_vel_mouse_callback(self, data):
        """
        Fake callback
        :param data:
        :return:
        """
        # print("cmd callback")


# TODO: outsource the following functions into util -> looked in moodle but didn't work

def update_state(state, omega, speed, iteration):
    """
    This function updates the state using the Euler method (see ex04)
    state = [x,y,z]
    :return:
    """
    iterations = 10
    dt = 1.0 / iterations
    current_state = copy.deepcopy(state)

    for i in range(0, iterations):
        changed_pos = np.array(
            [current_state[0],
             current_state[1],
             current_state[2]]
        ) + np.array(
            [
                [np.cos(current_state[2]), 0],
                [np.sin(current_state[2]), 0],
                [0, 1]]
        ) @ np.array([speed, omega]) \
                      * dt

        current_state[0] = changed_pos[0]
        current_state[1] = changed_pos[1]
        current_state[2] = changed_pos[2]

    return current_state


if __name__ == "__main__":
    rospy.init_node("mouse_nav")
    try:
        node = Mouse()
    except rospy.ROSInterruptException:
        pass
