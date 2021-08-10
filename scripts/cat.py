import numpy as np


class Cat:
    def __init__(self, cheese_array):
        self.cheese_array = cheese_array
        self.max_speed = 0
        self.max_angular = 0

    def update(self, cat_position, mouse_position, speed, angular):
        """Update all variables of interest.

        Args:
            cat_position (np.array): updated position of cat
            mouse_position (np.array): updated position of mouse
            speed (float): current speed of cat
            angular (float): current angular velocity of cat
        """
        self.position = cat_position
        self.mouse_position = mouse_position

        self.max_speed = speed if speed > self.max_speed else self.max_speed
        self.max_angular = angular if angular > self.max_angular else self.max_angular

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
        time = (distance / self.speed) * (self.speed / mouse_speed)

        # alternatively could return the time and work with this (if time to reach cheese < time for cat to catch mouse don't change plans)
        return True if time < threshold else False
