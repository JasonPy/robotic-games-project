#!/usr/bin/env python

import os
import rospy
import numpy as np
import actionlib
import rospkg

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from enum import Enum

catch_path = rospkg.RosPack().get_path('catch')


class MapType(Enum):
    MAP_1 = "Map_1"
    MAP_2 = "MAP_2"


def move_to_cheese(target_position):
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Move in X, Y and Z direction
    goal.target_pose.pose.position.x = target_position[0]
    goal.target_pose.pose.position.y = target_position[1]
    goal.target_pose.pose.position.z = target_position[2]

    rospy.loginfo(
        f"Target Location [x,y,z]: {goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z}")

    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()

    # If the result doesn't arrive, assume the Server is not available+
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Result of executing the action
        return client.get_result()


def get_cheese_positions(map_type: MapType) -> np.array:
    """
    load cheese contours for a specific map and return their center position

    :param map_type: the type of the current map
    :return: an array containing x,y positions of all cheese
    """
    cheese_contours = []
    ids = []

    if map_type == MapType.MAP_1:
        ids = [1, 2, 3, 4]
    if map_type == MapType.MAP_2:
        ids = [5, 6, 7, 8]

    for i in ids:
        filepath = os.path.join(catch_path, 'maps/cheese_' + str(i) + '.npy')
        cheese_contour = np.load(filepath)
        cheese_contours.append(np.mean(cheese_contour, axis=0))

    return np.array(cheese_contours)
