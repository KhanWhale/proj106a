from scipy.linalg import lstsq
import matplotlib.pyplot as plt
import numpy as np
from graph import *


base_axis1, base_axis2, base_axis3 = None, None, None

def init():
    global base_axis1
    base_axis1 = None
    global base_axis2
    base_axis2 = None
    global base_axis3
    base_axis3 = None


def get_angles(x_coords, y_coords, z_coords):
    # set up linear system
    ones = np.repeat(1, len(x_coords))
    A = np.concatenate((x_coords[:,np.newaxis], y_coords[:,np.newaxis], ones[:,np.newaxis]),axis=1)
    b = z_coords
    plane_coeffs, residual, rnk, s = lstsq(A, b)


    X,Y = np.meshgrid(x_coords, y_coords)
    Z = plane_coeffs[0] * X + plane_coeffs[1] * Y + plane_coeffs[2]

    best_fit_plane = np.array([X.flatten(), Y.flatten(), Z.flatten()])
    centroid = np.mean(best_fit_plane, axis=1, keepdims=True)
    svd = np.linalg.svd(best_fit_plane - centroid)
    print(base_axis1)
    if base_axis1 is None:
        base_axis1 = svd[0][:, -1]
    normal_vector = svd[0][:, -1] #left singular vector
    #normal_fn = lambda X,Y,Z: np.cross(np.array([X[1]-X[0], Y[1]-Y[0], Z[1]-Z[0]]), np.array([X[2]-X[0], Y[2]-Y[0], Z[2]-Z[0]]))

    origin = centroid.flatten()
    
    if base_axis2 is None:
        base_axis2 = np.array([x_coords[12], y_coords[12], z_coords[12]])
    axes2 = np.array([x_coords[12], y_coords[12], z_coords[12]])
    
    cross_prod_fn = lambda vec1,vec2: np.cross(vec1, vec2)
    if base_axis3 is None:
        base_axis3 = cross_prod_fn(normal_vector, axes2)
    axes3 = cross_prod_fn(normal_vector, axes2)

    angle_1 = np.arccos(np.dot(base_axis1-origin, normal_vector-origin) / (np.linalg.norm(base_axis1-origin) * np.linalg.norm(normal_vector-origin)))
    angle_2 = np.arccos(np.dot(base_axis2-origin, axes2-origin) / (np.linalg.norm(base_axis2-origin) * np.linalg.norm(axes2-origin)))
    angle_3 = np.arccos(np.dot(base_axis3-origin, axes3-origin) / (np.linalg.norm(base_axis3-origin) * np.linalg.norm(axes3-origin)))


    coords = [x_coords, y_coords, z_coords]
    axes = [normal_vector, axes2, axes3]
    plane_coords = [X, Y, Z]
    #graph_hand(coords, origin, axes)

    return angle_1, angle_2, angle_3
