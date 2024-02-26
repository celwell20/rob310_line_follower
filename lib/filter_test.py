import matplotlib.pyplot as plt

class LowPassFilter:
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

def generate_step_increase_speed(time_steps, initial_speed, step_increase, step_time):
    speed = [initial_speed + step_increase if t >= step_time else initial_speed for t in range(time_steps)]
    return speed

def plot_speed_comparison(current_speed, filtered_speed):
    plt.plot(current_speed, label='Current Speed')
    plt.plot(filtered_speed, label='Filtered Speed')
    plt.xlabel('Time Steps')
    plt.ylabel('Speed')
    plt.legend()
    plt.title('Current Speed vs. Filtered Speed')
    plt.show()

# Parameters
time_steps = 100
initial_speed = 5.0
step_increase = 3.0
step_time = 30
alpha_value = 0.2  # Adjust this value based on the level of smoothing you desire

# Generate step increase in speed
current_speed = generate_step_increase_speed(time_steps, initial_speed, step_increase, step_time)

# Filter the speed using a low-pass filter
speed_filter = LowPassFilter(alpha_value)
filtered_speed = [speed_filter.update(speed) for speed in current_speed]

# Plot the results
plot_speed_comparison(current_speed, filtered_speed)
