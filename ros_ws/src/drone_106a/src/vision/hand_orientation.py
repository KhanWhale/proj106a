from scipy.linalg import lstsq
import matplotlib.pyplot as plt
import numpy as np
from graph import *
import rospy

base_axis1, base_axis2, base_axis3 = None, None, None

def get_angles(x_coords, y_coords, z_coords):
    global base_axis1
    global base_axis2
    global base_axis3
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
    if base_axis1 is None:
        base_axis1 = svd[0][:, -1]
    normal_vector = svd[0][:, -1] #left singular vector
    #normal_fn = lambda X,Y,Z: np.cross(np.array([X[1]-X[0], Y[1]-Y[0], Z[1]-Z[0]]), np.array([X[2]-X[0], Y[2]-Y[0], Z[2]-Z[0]]))

    origin = centroid.flatten()
    
    if base_axis2 is None:
        base_axis2 = np.array([x_coords[12], y_coords[12], z_coords[12]])
    axes2 = np.array([x_coords[12], y_coords[12], z_coords[12]])
    
    cross_prod_fn = lambda u,v: np.cross(u, v)
    if base_axis3 is None:
        base_axis3 = cross_prod_fn(normal_vector, axes2)
    axes3 = cross_prod_fn(normal_vector, axes2)
    try:
        angle_1 = np.arccos(np.dot(base_axis1-origin, normal_vector-origin) / (np.linalg.norm(base_axis1-origin) * np.linalg.norm(normal_vector-origin)))
    except: 
        # rospy.loginfo('Invalid angle_1')
        print('INvalid angle_1')
        angle_1 = 0
    cross1 = cross_prod_fn(base_axis1, normal_vector)
    if np.dot(cross1, axes2) < 0:
        angle_1 = -angle_1
    try:
        angle_2 = np.arccos(np.dot(base_axis2-origin, axes2-origin) / (np.linalg.norm(base_axis2-origin) * np.linalg.norm(axes2-origin)))
    except:
        # rospy.loginfo('Invalid angle_2')
        print('INvalid angle_2')
        angle_2 = 0
    cross2 = cross_prod_fn(base_axis2, axes2)
    if np.dot(normal_vector, cross2) < 0:
        angle_2 = -angle_2
    try:
        angle_3 = np.arccos(np.dot(base_axis3-origin, axes3-origin) / (np.linalg.norm(base_axis3-origin) * np.linalg.norm(axes3-origin)))
    except:
        # rospy.loginfo('Invalid angle_3')
        print('INvalid angle_3')
        angle_3 = 0
    cross3 = cross_prod_fn(base_axis3, axes3)
    if np.dot(normal_vector, cross3) < 0:
        angle_3 = -angle_3

    #coords = [x_coords, y_coords, z_coords]
    #axes = [normal_vector, axes2, axes3]
    #plane_coords = [X, Y, Z]
    #graph_hand(coords, origin, axes)
    angle_1 = 0 if np.isnan(angle_1) else angle_1
    angle_2 = 0 if np.isnan(angle_2) else angle_2
    angle_3 = 0 if np.isnan(angle_3) else angle_3
    return int(angle_1*(180/np.pi)), int(angle_2*(180/np.pi)), int(angle_3*(180/np.pi))
