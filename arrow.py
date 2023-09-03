# arrow
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform
from mpl_toolkits.mplot3d.axes3d import Axes3D
from transform import *

class Arrow3D(FancyArrowPatch):
    
    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)
        
    def do_3d_projection(self, renderer=None):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))

        return np.min(zs)
        
def _arrow3D(ax, x, y, z, dx, dy, dz, *args, **kwargs):
    '''Add an 3d arrow to an `Axes3D` instance.'''

    arrow = Arrow3D(x, y, z, dx, dy, dz, *args, **kwargs)
    ax.add_artist(arrow)

setattr(Axes3D, 'arrow3D', _arrow3D)

def visualize_posture(p, R, ax, elems):
    scale = 1.0
    ex = R[:,0] * scale
    ey = R[:,1] * scale
    ez = R[:,2] * scale
    # ex
    ax.arrow3D(    p[0,0],p[1,0],p[2,0],
                            ex[0],ex[1],ex[2],
                            mutation_scale=20,
                            ec ='red',
                            fc='red')
    # ey
    ax.arrow3D(    p[0,0],p[1,0],p[2,0],
                            ey[0],ey[1],ey[2],
                            mutation_scale=20,
                            ec ='green',
                            fc='green')
    # ez
    ax.arrow3D(    p[0,0],p[1,0],p[2,0],
                            ez[0],ez[1],ez[2],
                            mutation_scale=20,
                            ec ='blue',
                            fc='blue')
    return



# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlim(0,2)
# # ax.arrow3D(0,0,0,
# #            1,1,1,
# #            mutation_scale=20,
# #            arrowstyle="-|>",
# #            linestyle='dashed')
# # ax.arrow3D(1,0,0,
# #            1,1,1,
# #            mutation_scale=20,
# #            ec ='red',
# #            fc='red')
# ax.set_title('3D Arrows Demo')
# p = np.array([0.5, 0.5, 0]).reshape(-1, 1)
# elems = []


# for i in range(100):
#     plt.cla()
#     theta = np.radians(i * 10) 
#     R = Rz(theta)
#     visualize_posture(p, R, ax, elems)
#     plt.pause(0.001)

# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# fig.tight_layout()
# plt.show()