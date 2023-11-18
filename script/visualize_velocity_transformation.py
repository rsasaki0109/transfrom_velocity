import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

with open('velocity_data.txt') as f:
    data = f.readlines()

Tba = np.array([list(map(float, line.split())) for line in data[1:5]])
va = np.array(list(map(float, data[6].split()[3:])))
vb = np.array(list(map(float, data[7].split()[3:])))

print('v = (vx vy vz wx wy wz)')
print('Tba')
print(Tba)
print('va')
print(va)
print('vb')
print(vb)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.quiver(0, 0, 0, va[0], va[1], va[2], color='r', label='Original Velocity (va)')

ax.quiver(Tba[0, 3], Tba[1, 3], Tba[2, 3], vb[0], vb[1], vb[2], color='b', label='Transformed Velocity (vb)')

ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.legend()

ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])

# グラフ表示
plt.show()