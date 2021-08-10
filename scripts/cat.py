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
