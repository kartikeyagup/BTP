import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

a="""[26900.3, 15479.8, 15820.3];[26013, 7401.46, 15300.8];[25781.2, -166.942, 15165.1];[26027.6, -7742.37, 15309.4];[26928.4, -15843.2, 15836.8];
[39250.3, 18477.6, 22979.9];[38495.7, 8966.71, 22539.3];[38229.5, -190.844, 22384];[38509.4, -9354.21, 22547.4];[39283.2, -18884.4, 22999.1];
[54227.5, 21625.1, 31653.1];[53272.7, 10515.8, 31097];[52932.9, -210.422, 30899.1];[53289.4, -10942.6, 31106.7];[54262.6, -22069.5, 31673.6];
[71270.5, 24667.2, 41512.8];[70184.5, 12023.9, 40881.4];[69782.8, -225.678, 40647.7];[70206.3, -12481.5, 40894];[71309.8, -25141.4, 41535.7];
[90872.5, 27779.5, 52846];[89833.4, 13612.7, 52242.5];[89464.5, -238.597, 52028.2];[89853.3, -14095.1, 52254.1];[90907.8, -28275.1, 52866.5];"""

points = ''.join(a.split('\n')).split(';')[:-1]
points = map(lambda x:map(float, x[1:-1].split(',')),points)
points = zip(*points)
print points
# exit()


def randrange(n, vmin, vmax):
    return (vmax - vmin)*np.random.rand(n) + vmin

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
n = 100

ax.scatter(points[0],points[1],points[2])

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
