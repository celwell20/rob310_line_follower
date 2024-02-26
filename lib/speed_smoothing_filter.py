class SmoothingFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.filtered_speed = None

    def update(self, current_speed):
        if self.filtered_speed is None:
            # Initialize the filtered speed with the current speed for the first iteration
            self.filtered_speed = current_speed
        else:
            # Update the filtered speed using the low-pass filter equation
            self.filtered_speed = (1 - self.alpha) * self.filtered_speed + self.alpha * current_speed

        return self.filtered_speed