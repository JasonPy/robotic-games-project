#! /usr/bin/env python
import copy
import numpy as np


def buildTree(self, nodeIndex, scoreTree, depth):
    # a few explanations to how the tree is build/structured:
    # since we have n choices and a depth of m we have int((self.choices ** (self.depth + 1) - 1) / 2) nodes
    # every node consists of an np-array which has the same shape as the game_state
    # numbers run within a level from left to right
    # (example: 3 choices and 3 levels ~ 0 = current game_state (level 0 of the tree), 1,2,3 (nodes in level 1),
    # 4,5,6,7,8,9,10,11,12 (nodes in level 2) and 13-39 (nodes in level 3)
    if depth == 0:
        return scoreTree
    for i in range(0, self.choices):

        if depth % 2 == 1:  # mouse case
            scoreTree[nodeIndex * self.choices + i + 1] = update_game_state(scoreTree[nodeIndex],
                                                                            self.strategy_choices_self[i],
                                                                            self.speed,
                                                                            True)
            # important: changed reward(scoreTree[...+ i]) to reward(scoreTree[...+ i + 1]) since we want to
            # calculate the reward for the new state
            scoreTree[nodeIndex * self.choices + i + 1][6] = reward(scoreTree[nodeIndex * self.choices + i + 1],
                                                                    True)

            scoreTree[nodeIndex * self.choices + i + 1][7] = nodeIndex * self.choices + i + 1

            # TODO: idea ~ we cannot walk in "every" direction due to collisions -> pruning or: utility combine
            # -> don't have to investigate these branches anymore

        else:  # cat case
            scoreTree[nodeIndex * self.choices + i + 1] = update_game_state(scoreTree[nodeIndex],
                                                                            self.strategy_choices_cat[i],
                                                                            self.speed_cat,
                                                                            False)
            # important: changed reward(scoreTree[...+ i]) to reward(scoreTree[...+ i + 1]) since we want to
            # calculate the reward for the new state
            scoreTree[nodeIndex * self.choices + i + 1][6] = reward(scoreTree[nodeIndex * self.choices + i + 1],
                                                                    False)
            scoreTree[nodeIndex * self.choices + i + 1][7] = nodeIndex * self.choices + i + 1

        scoreTree = self.buildTree(nodeIndex * self.choices + i + 1, scoreTree, depth - 1)
    return scoreTree


def minimax(nodeIndex, maximize, scoreTree, depth, choices):
    # base case : targetDepth reached

    if depth == 0:
        return scoreTree[nodeIndex]

    if maximize:
        value = [0, 0, 0, 0, 0, 0, -np.inf]
        for i in range(0, choices):
            if value[6] < minimax(nodeIndex * choices + i + 1, False, scoreTree, depth - 1, choices)[6]:
                value = minimax(nodeIndex * choices + i + 1, False, scoreTree, depth - 1, choices)
        return value
    else:  # minimum
        value = [0, 0, 0, 0, 0, 0, np.inf]
        for i in range(0, choices):
            if value[6] > minimax(nodeIndex * choices + i + 1, False, scoreTree, depth - 1, choices)[6]:
                value = minimax(nodeIndex * choices + i + 1, False, scoreTree, depth - 1, choices)
        return value


def update_state(state, omega, speed, iterations):
    """
    This function updates the state using the Euler method (see ex04)
    state = [x,y,z]
    :return:
    """
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


# TODO: rethink reward calculation
def reward(game_state, is_mouse):
    """
    This function assigns a reward to a game_state for the minimax-algorithm
    :param state: game_state
    :param is_mouse: boolean which indicates for which player the reward is determined
    :return: reward
    """
    # TODO: used reward function of the sample solution -> check that (is that really a good idea?)
    distance = np.sqrt((game_state[0] - game_state[3]) ** 2 + (game_state[1] - game_state[4]) ** 2)
    alpha = game_state[2] - game_state[5]
    beta = np.arctan2((game_state[1] - game_state[4]), (game_state[0] - game_state[3])) - game_state[5]
    # print(alpha, " ", beta)
    value = distance * (np.cos(alpha / 2) + np.sin(beta / 2))

    return value


def update_game_state(game_state, omega, speed, is_mouse):
    """
    This function updates the game state for the tree
    :return:
    """
    current_state = copy.deepcopy(game_state)
    if not is_mouse:  # cat case
        # call update_state function with params for the cat
        current_state[3:6] = update_state(current_state[3:6], omega, speed, 10)
        # the rest of the game_state stays the same

    else:  # mouse case
        # call update_state function with params for the mouse
        current_state[0:3] = update_state(current_state[0:3], omega, speed, 10)

    return current_state
