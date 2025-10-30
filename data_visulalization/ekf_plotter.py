"""
ekf_plotter.py

A subscriber that visualizes the EKF's orientation data as four
real-time probability distributions (normal curves).
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import threading
from orientation_mediator import OrientationMediator

class GaussianPlotter:
    """
    Subscribes to the OrientationMediator and plots the
    probability density function (PDF) for each quaternion component.
    """

    def __init__(self, mediator: OrientationMediator):
        self.mediator = mediator
        self.mediator.subscribe(self.update_data)
        
        self.latest_data = None
        self.lock = threading.Lock()
        
        # Set up the plot
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('Real-Time EKF Quaternion Distributions', fontsize=16)
        
        # Define the x-axis range (quaternion values are -1 to 1)
        self.x_range = np.linspace(-1.2, 1.2, 300)
        
        # Titles for each subplot
        titles = ['q_x Distribution', 'q_y Distribution', 'q_z Distribution', 'q_w Distribution']
        self.lines = []
        
        for i, ax in enumerate(self.axs.flat):
            # Create an initial empty line
            line, = ax.plot(self.x_range, np.zeros_like(self.x_range), lw=2)
            self.lines.append(line)
            ax.set_title(titles[i])
            ax.set_xlim(-1.2, 1.2)
            ax.grid(True)
        
        # Set labels
        self.axs[1, 0].set_xlabel('Quaternion Value')
        self.axs[1, 1].set_xlabel('Quaternion Value')
        self.axs[0, 0].set_ylabel('Probability Density')
        self.axs[1, 0].set_ylabel('Probability Density')

    def update_data(self, data):
        """Callback function to receive new orientation data."""
        with self.lock:
            self.latest_data = data
            
    def _normal_dist(self, x, mu, sigma):
        """Calculates the probability density function for a normal distribution."""
        # Prevent division by zero if variance is tiny
        if sigma < 1e-6:
            sigma = 1e-6
        return (1.0 / (sigma * np.sqrt(2 * np.pi))) * \
               np.exp(-0.5 * ((x - mu) / sigma) ** 2)

    def _animate(self, i):
        """The animation update function."""
        with self.lock:
            local_data = self.latest_data
            
        if local_data is None:
            return self.lines
            
        # Get means (the quaternion values)
        means = [local_data.qx, local_data.qy, local_data.qz, local_data.qw]
        
        # Get variances
        variances = [local_data.var_qx, local_data.var_qy, 
                       local_data.var_qz, local_data.var_qw]
        
        max_y = 0 # To dynamically scale the y-axis
        
        for k in range(4):
            mu = means[k]
            sigma = np.sqrt(variances[k]) # Convert variance to std. deviation
            
            # Calculate the new y-data for the curve
            y_data = self._normal_dist(self.x_range, mu, sigma)
            
            # Update the line
            self.lines[k].set_ydata(y_data)
            
            # Find the peak of this curve for y-axis scaling
            peak_y = self._normal_dist(mu, mu, sigma)
            if peak_y > max_y:
                max_y = peak_y
        
        # Update all axes to have the same, dynamic y-height
        for ax in self.axs.flat:
            ax.set_ylim(0, max_y * 1.15 + 1e-3) # Add a small buffer
            
        return self.lines

    def run(self):
        """
        Starts the matplotlib animation.
        This function will block until the plot window is closed.
        """
        # 'blit=True' means only re-draw the parts that have changed
        # 'interval=50' aims for ~20 FPS (50ms)
        ani = animation.FuncAnimation(self.fig, self._animate, 
                                      interval=50, blit=True)
        
        plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust for suptitle
        plt.show() # This blocks the main thread
