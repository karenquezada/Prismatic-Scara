import numpy as np

def create_points(start, end, n):
    # Si el punto inicial y final tienen diferentes signos en x, crea dos rectas simétricas
    if np.sign(start[0]) != np.sign(end[0]):
        x1 = np.linspace(start[0], 0, n // 2)
        x2 = np.linspace(0, end[0], n // 2)
        y1 = np.linspace(start[1], 0, n // 2)
        y2 = np.linspace(0, end[1], n // 2)
        points1 = np.column_stack((x1, y1))
        points2 = np.column_stack((x2, y2))
        points = np.concatenate((points1, points2))
    else:
        x = np.linspace(start[0], end[0], n)
        y = np.linspace(start[1], end[1], n)
        points = np.column_stack((x, y))
    return points

start = [-100, -10, 100]
end = [200, 335, 100]
n = 30

points = create_points(start, end, n)

import matplotlib.pyplot as plt

plt.scatter(points[:, 0], points[:, 1])
plt.xlabel('X')
plt.ylabel('Y')
plt.xlim(-400, 400)
plt.ylim(-300, 500)

plt.show()
