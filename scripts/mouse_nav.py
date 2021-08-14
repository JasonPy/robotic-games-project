#! /usr/bin/env python

import copy
import rospy
import numpy as np
import random
import rogata_library.rogata_library as rgt

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

import util
import minmax
import gates
import cat

rgt_helper = rgt.rogata_helper()
game_state = np.zeros(8)  # game_state is used for building the tree/minimax-algorithm

# HYPERPARAMETER
CAT_MOUSE_MAX_DIST = 100


class Mouse:

    def __init__(self):
        # positions of cheese objects (have to be initially loaded since cheese_obj only contains the mean
        # cheese coordinate)
        self.cheese_amount = 4
        # x,y pos of target cheese and mouse
        self.target_cheese = 0
        self.position = np.array([0, 0, 0])
        self.speed = 0.18 + random.uniform(0, 0.4)
        self.angular_speed = 2 + random.uniform(-0.4, 0.4)

        # cat
        self.cat = Cat(self.cheese_array)

        # parameters for the tree planning
        self.choices = 3
        self.depth = 5  # important: depth needs to be odd - see exercise sheet
        self.game_state = np.zeros(8)
        # game_state consists of
        # [x_cat, y_cat, rot_cat, x_mouse, y_mouse, rot_mouse, reward_for_player, pos_in_tree]

        # attributes for collision avoidance and free space
        self.omega_ca = 0
        self.omega_fs = 0
        self.angles = np.array([])
        self.ranges = np.array([])

        # callback for mouse and cat position
        rospy.Subscriber("mouse_obj/odom", Odometry, self.odom_mouse_callback)
        rospy.Subscriber("cat_obj/odom", Odometry, self.odom_cat_callback)
        rospy.Subscriber("mouse/cmd_vel", Twist, self.cmd_vel_mouse_callback)  # fake subscriber
        rospy.Subscriber("mouse/ca_cmd_vel", Twist, self.ca_cmd_vel_callback)  # collision_avoidance
        rospy.Subscriber("mouse/fs_cmd_vel", Twist, self.fs_cmd_vel_callback)  # collision_avoidance
        rospy.Subscriber("mouse/scan", LaserScan, self.scan_callback)

        # publisher for mouse velocities
        self.pub = rospy.Publisher("mouse/cmd_vel", Twist, queue_size=10)  # or mouse_cmd_vel?

        print("--------------- game is starting ---------------")

        while not rospy.is_shutdown():

            if self.cat_is_in_near_mouse(threshold=CAT_MOUSE_MAX_DIST):
                # if cat enters mouse cell -> flee using minimax-approach
                # TODO discretize decision space correctly/in such a way that it makes sense
                # discretized decision spaces to build the tree
                self.strategy_choices_self = np.linspace(-self.angular_speed, self.angular_speed, self.choices)
                self.strategy_choices_cat = np.linspace(-2.84, 2.84, self.choices)
                self.speed_cat = 0.4  # we don't know that

                omega_minimax = self.get_minimax_omega()

                # combine omega of minimax with collision avoidance
                omega_combined = self.combine_omegas(omega_minimax)
                output = Twist()
                output.linear.x = self.speed  # fixed, we only consider the angular velocity
                output.angular.z = omega_minimax
                self.pub.publish(output)
                # TODO: try 2nd approach: integrate collision avoidance into minimax (pruning)

            else:
                # determine next best move in the 'absence' of the cat -> optimal homing + collision avoidance,
                # then: think of what the cat might be doing (while the cat is also thinking about what we might
                # be doing)
                print("go to the cheese")
                # homing/navigation/collision avoidance

        self.cheese_array = get_cheese_positions("MAP_1")
        # why are we using a loop instead of rospy.spin() -- what is that for?
        # rospy.spin() effectively goes into an infinite loop until it receives a shutdown signal (e.g. ctrl-c).
        # During that loop it will process any events that occur, such as data received on a topic or a timer
        # triggering, when they occur but otherwise it will sleep. You should use spin() when your node doesn't do
        # anything outside of its callbacks. -> we don't need rospy.spin() but the while loop

    # TODO: important: these functions are taken from the sample solution of ex07-> have to be checked!
    def get_minimax_omega(self):
        scoreTree = [None] * int((self.choices ** (self.depth + 1) - 1) / 2)  # create empty tree
        scoreTree[0] = copy.deepcopy(game_state)
        # print(f"tree is being build with {int((self.choices ** (self.depth + 1) - 1) / 2)} nodes")
        scoreTree = buildTree(0, scoreTree,
                              self.depth)  # self.depth-1 instead of self.depth? don't think sp
        # print(f"scoreTree is built: {scoreTree}")
        best_choice = minimax(0, True, scoreTree, self.depth, self.choices)  # self.depth - 1? don't think so
        # print(f"best choice before while loop {best_choice}")
        while best_choice[7] > self.choices:
            best_choice[7] = int(best_choice[7] / self.choices)

        # print (best_choice)

        return self.strategy_choices_self[int(best_choice[7] - 1)]

    def cheese_callback(self, data):
        # evaluate all cheese
        for cheese in self.cheese_array:
            cheese.evaluate(self.position, self.cat.position,
                            self.speed, self.cat.speed)

        # sort array by score
        self.cheese_array(key=lambda x: x.score, reverse=True)

        # get coordinates of cheese with best evaluation score
        x, y = self.cheese_array[0].position

        self.target_cheese = np.array([x, y])
        print(f"target cheese: {self.target_cheese}")

    # TODO
    def get_flee_from_cat_score(self):
        """
        Determine how urgent it is too flee from the cat.
        :return: 0-1, 1 = 100% flee, 0 = 0% flee (so go to the cheese)
        """

    def cat_is_in_near_mouse(self, threshold):
        """
        This function determines the distance between cat and mouse
        and returns True if their distance is below some threshold.
        :return: (bool) if cat is near mouse
        """
        distance = rgt_helper.dist("cat_obj", rgt_helper.get_pos("mouse_obj"))
        return True if distance < threshold else False

    # TODO: update target_cheese?
    def odom_mouse_callback(self, data):
        orientation = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])[2]
        x_pos = data.pose.pose.position.x
        y_pos = data.pose.pose.position.y

        self.position = np.array([x_pos, y_pos, orientation])
        game_state[0] = x_pos
        game_state[1] = y_pos
        game_state[2] = orientation

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

        game_state[3] = x_pos
        game_state[4] = y_pos
        game_state[5] = orientation
        # self.update_target_cheese() ?
        self.cat.update(np.array([x_pos, y_pos, orientation]))  # update cat

    def ca_cmd_vel_callback(self, msg):  # new command from collision avoidance node
        self.omega_ca = msg.angular.z

    def fs_cmd_vel_callback(self, msg):  # new command from free space node
        self.omega_fs = msg.angular.z

    def cmd_vel_mouse_callback(self, data):
        """
        Fake callback
        :param data:
        :return:
        """
        # print("cmd callback")

    def scan_callback(self, msg):
        ranges = msg.ranges
        ranges = np.array(ranges)
        angle_min = msg.angle_min  # start angle of the scan [rad]
        angle_max = msg.angle_max  # end angle of the scan [rad]
        angle_increment = msg.angle_increment  # angular distance between measurements [rad]
        angles = np.arange(angle_min, angle_max + angle_increment, angle_increment)

        self.angles = angles[np.isfinite(ranges)]  # delete 'inf'
        self.ranges = ranges[np.isfinite(ranges)]

    def combine_omegas(self, omega_minimax):
        # normalize omegas to [-1, 1]
        norm_omega_ca = normalize(self.omega_ca)
        norm_omega_fs = normalize(self.omega_fs)
        norm_omega_minimax = normalize(omega_minimax)

        omega = PREVAIL(norm_omega_ca, PREVAIL(norm_omega_minimax, norm_omega_fs))
        return omega * np.pi  # get back angular vel [rad]


if __name__ == "__main__":
    rospy.init_node("mouse_nav")
    try:
        node = Mouse()
    except rospy.ROSInterruptException:
        pass
