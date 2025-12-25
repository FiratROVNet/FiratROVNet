import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt

points = np.array([
    [1, 1],
    [2, 5],
    [3, 3],
    [5, 3],
    [3, 2],
    [2, 2]
])

hull = ConvexHull(points)

plt.scatter(points[:,0], points[:,1])
for simplex in hull.simplices:
    plt.plot(points[simplex,0], points[simplex,1], 'r-')

plt.show()
