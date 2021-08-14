import numpy as np


class Cat:
    def __init__(self, cheese_array):
        self.cheese_array = cheese_array
        self.position = np.zeros(2)
        self.max_speed = 0.3  # smallest possible value = 0.2, highest possible value 0.4
        self.max_angular = 1  # smallest possible value = -2, highest possible value 2

    def update(self, cat_position):
        """Update all variables of interest.

        Args:
            cat_position (np.array): updated position of cat
        """
        velocities = self.calculate_speed(cat_position)
        speed = velocities[0]
        angular = velocities[1]

        self.position = cat_position

        self.max_speed = speed if speed > self.max_speed else self.max_speed
        self.max_angular = angular if angular > self.max_angular else self.max_angular

    def calculate_speed(self, new_position):
        """
        This function estimates the angular and linear velocity of the cat given the prior and the current position.
        :param new_position: current position of the cat
        """
        velocities = np.zeros(2)
        return velocities

    def plan_movement(self):
        "plan next moves of cat"
        pass

    def cat_near_mouse(self, mouse_position, mouse_speed, threshold):
        """Is the cat close the the mouse and warrants a change of plans

        Args:
            mouse_position (np.array): mouse position
            mouse_speed (float): maxspeed of the mouse
            threshold (float): time threshold to consider cat close

        Returns:
            bool: bool representing "closeness"
        """
        distance = np.linalg.norm(mouse_position - self.position)
        time = (distance / self.max_speed) * (self.max_speed / mouse_speed)

        # alternatively could return the time and work with this (if time to reach cheese < time for cat to catch mouse don't change plans)
        return True if time < threshold else False
