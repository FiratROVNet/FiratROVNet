import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D




import numpy as np
from scipy.spatial import ConvexHull

points = np.array([
    [0,0,0],
    [1,0,0],
    [1,1,0],
    [0,1,0],
    [0,0,1],
    [1,1,1]
])

hull = ConvexHull(points)





fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')



ax.scatter(points[:,0], points[:,1], points[:,2])

for simplex in hull.simplices:
    simplex = np.append(simplex, simplex[0])
    ax.plot(points[simplex,0],
            points[simplex,1],
            points[simplex,2], 'r-')

plt.show()
