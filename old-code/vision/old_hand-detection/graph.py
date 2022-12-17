import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def graph_hand(coords, origin, axes):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(coords[0], coords[1], coords[2], color='g')
    ax.quiver(origin[0], origin[1], origin[2], axes[0][0], axes[0][1], axes[0][2])
    ax.quiver(origin[0], origin[1], origin[2], axes[1][0], axes[1][1], axes[1][2])
    ax.quiver(origin[0], origin[1], origin[2], axes[2][0], axes[2][1], axes[2][2])
    #ax.plot_surface(X, Y, Z)
    set_axes_equal(ax)
    #plt.show()
    

def set_axes_radius(ax, origin, radius):
    '''
        From StackOverflow question:
        https://stackoverflow.com/questions/13685386/
    '''
    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])


def set_axes_equal(ax, zoom=1.):
    '''
        Make axes of 3D plot have equal scale so that spheres appear as spheres,
        cubes as cubes, etc..  This is one possible solution to Matplotlib's
        ax.set_aspect("equal") and ax.axis("equal") not working for 3D.
        input:
          ax:   a matplotlib axis, e.g., as output from plt.gca().

    '''

    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])

    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0])) / zoom
    set_axes_radius(ax, origin, radius)