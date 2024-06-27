import numpy as np
import matplotlib.pyplot as plt

def create_arc(start, end, center, n):
    start_angle = np.arctan2(start[1] - center[1], start[0] - center[0])
    end_angle = np.arctan2(end[1] - center[1], end[0] - center[0])
    angles = np.linspace(start_angle, end_angle, n)
    radius = np.sqrt((start[0] - center[0])**2 + (start[1] - center[1])**2)
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    z = np.linspace(start[2], end[2], n)
    points = np.column_stack((x, y, z))
    points[-1] = end
    print("points: ", points)
    return points

# Define the start, end, and center points
start = np.array([20, 230, 250])
end = np.array([10,-250,250])
center = np.array([0, 0, 70])
n=10
# Call the create_arc function to get the points
points = create_arc(start, end, center, n)

# Extract x, y, and z coordinates from the points
x = points[:, 0]
y = points[:, 1]
z = points[:, 2]

# Plot the points
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Arc Points')

# Show the plot
plt.show()