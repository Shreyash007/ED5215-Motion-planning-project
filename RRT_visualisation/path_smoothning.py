import numpy as np
import matplotlib.pyplot as plt

def smooth_path(points, window_size=3):
    # Convert points to a numpy array
    points = np.array(points)
    
    # Apply a moving average filter to x and y coordinates separately
    weights = np.repeat(1.0, window_size) / window_size
    print(weights)
    smoothed_x = np.convolve(points[:,0], weights, mode='valid')
    smoothed_y = np.convolve(points[:,1], weights, mode='valid')
    
    # Combine smoothed x and y coordinates into a single array
    smoothed_points = np.vstack((smoothed_x, smoothed_y)).T
    
    return smoothed_points

# Example usage:
points = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5),(5,7),(8,8)]
smoothed_points = smooth_path(points, window_size=3)

# Extract x and y coordinates from points list using list comprehension
x_coords = [point[0] for point in points]
y_coords = [point[1] for point in points]

# Extract x and y coordinates from smoothed points using numpy array indexing
smoothed_x = smoothed_points[:, 0]
smoothed_y = smoothed_points[:, 1]

# Plot the original and smoothed path
plt.plot(x_coords, y_coords, 'bo-', label='Original Path')
plt.plot(smoothed_x, smoothed_y, 'r.-', label='Smoothed Path')
plt.legend()
plt.show()
