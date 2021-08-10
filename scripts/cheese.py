#! /usr/bin/env python3
# license removed for brevity

import numpy as np


class Cheese:
    "Holds the position and score of a cheese"

    def __init__(self, position):
        self.position = position
        self.score = 0

    def evaluate(self, mouse_position, cat_position):
        """Evaluate Cheese based on mouse_position, cat_position

        Args:
            mouse_position (np.array): x and y coordinates of mouse
            cat_position (np.array): x and y coordinates of cat
        """
        # euclidian distance from cat and mouse to cheese
        mouse_proximity = np.linalg.norm(self.position - mouse_position)
        cat_proximity = np.linalg.norm(self.position - cat_position)

        # euclidian distance from cat to mouse
        distance_mouse_cat = np.linalg.norm(mouse_position - cat_position)

        # angle between mouse and cat
        angle_mouse_cat = np.arccos(
            (cat_proximity ** 2 + mouse_proximity ** 2 - distance_mouse_cat ** 2)
            / (2 * cat_proximity * mouse_proximity))

        # utility function to evaluate cheese
        self.score = mouse_proximity * (
            np.clip(cat_proximity / mouse_proximity, 0, 1) * (angle_mouse_cat / 180))
