import numpy as np
import matplotlib.pyplot as plt

def create_arc(start, end, n):
    center = (start + end) / 2
    start_angle = np.arctan2(start[1] - center[1], start[0] - center[0])
    end_angle = np.arctan2(end[1] - center[1], end[0] - center[0])
    angles = np.linspace(start_angle, end_angle, n)
    radius = np.sqrt((start[0] - center[0])**2 + (start[1] - center[1])**2)
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    points = np.column_stack((x, y))
    points[-1] = end
    return points

# Define the start, end, and center points
punto2 = np.array([100, -50])
punto1 = np.array([176.24, 39.18])
n=8
# Call the create_arc function to get the points
points = create_arc(punto1, punto2, n)

# Extract x, y, and z coordinates from the points
x = points[:, 0]
y = points[:, 1]

# Plot the points
fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(x, y)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Arc Points')

# Show the plot
plt.show()