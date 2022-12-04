import os
import sys
import time
import cv2
import numpy as np
import mediapipe as mp
import handdetection
import pyrealsense2 as rs
from scipy.linalg import lstsq
import matplotlib.pyplot as plt

pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
	if s.get_info(rs.camera_info.name) == 'RGB Camera':
		found_rgb = True
		break
if not found_rgb:
	print("The demo requires Depth camera with Color sensor")
	exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
	config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
	config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

align_to = rs.stream.color
align = rs.align(align_to)

handdetection.init()

# FPS calculation initialization
start_time = time.time()
fps_calc_window = 1
num_frames = 0
fps = 0
base_frame = False
base_axis1, base_axis2, base_axis3 = None, None, None
from graph import *

try:
	while True:
		x_coords = np.array([])
		y_coords = np.array([])
		z_coords = np.array([])
		if not base_frame:
			print("\n\n\n *************** Smart Jedi X-Wing Starfighter *************** \n\nPlease place your hand in the frame to initialize the base frame\n\n")
			#cv2.waitKey(10)
			time.sleep(3)
			base_frame=True
		frames = pipeline.wait_for_frames()
		aligned_frames = align.process(frames)

		aligned_depth_frame = aligned_frames.get_depth_frame()
		color_frame = aligned_frames.get_color_frame()

		# Validate that both frames are valid
		if not aligned_depth_frame or not color_frame:
			continue

		depth_image = np.asanyarray(aligned_depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())
		image_height, image_width, _ = color_image.shape

		processed_img, hand_landmarks, gesture = handdetection.process(color_image)
		if hand_landmarks:
			landmarks = hand_landmarks.landmark

			for i in range(21): # keypoint in mp.solutions.hands.HandLandmark
				pixel_x, pixel_y = landmarks[i].x * image_width, landmarks[i].y * image_height
				rounded_x, rounded_y = round(pixel_x), round(pixel_y)
				depth = 0
				if rounded_x >= 0 and rounded_x < 480 and rounded_y >= 0 and rounded_y < 480:
					depth = depth_image[rounded_y][rounded_x]
				#z_coords = np.append(z_coords, depth)
				print(f"{mp.solutions.hands.HandLandmark(i)}: (X: {pixel_x}, Y: {pixel_y}, Depth: {depth})")
			print("\n\n")

			#z_coords[z_coords == 0] = np.mean(z_coords)
			#z_coords = z_coords

			x_coords = np.append(x_coords, np.array([hand_landmarks.landmark[i].x for i in range(21)]))
			y_coords = np.append(y_coords, np.array([hand_landmarks.landmark[i].y for i in range(21)]))
			z_coords = np.append(z_coords, np.array([hand_landmarks.landmark[i].z for i in range(21)]))

			# set up linear system
			ones = np.repeat(1, len(x_coords))
			A = np.concatenate((x_coords[:,np.newaxis], y_coords[:,np.newaxis], ones[:,np.newaxis]),axis=1)
			b = z_coords
			plane_coeffs, residual, rnk, s = lstsq(A, b)

			fig = plt.figure()
			ax = fig.add_subplot(111, projection='3d')
			ax.scatter(x_coords, y_coords, z_coords, color='g')

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
			ax.quiver(origin[0], origin[1], origin[2], normal_vector[0], normal_vector[1], normal_vector[2])
			if base_axis2 is None:
				base_axis2 = np.array([hand_landmarks.landmark[12].x, hand_landmarks.landmark[12].y, hand_landmarks.landmark[12].z])
			axes2 = np.array([hand_landmarks.landmark[12].x, hand_landmarks.landmark[12].y, hand_landmarks.landmark[12].z])
			ax.quiver(origin[0], origin[1], origin[2], axes2[0], axes2[1], axes2[2])
			cross_prod_fn = lambda vec1,vec2: np.cross(vec1, vec2)
			if base_axis3 is None:
				base_axis3 = cross_prod_fn(normal_vector, axes2)
			axes3 = cross_prod_fn(normal_vector, axes2)

			os.system('clear')

			angle_1 = np.arccos(np.dot(base_axis1-origin, normal_vector-origin) / (np.linalg.norm(base_axis1-origin) * np.linalg.norm(normal_vector-origin)))
			print("\n Angle 1:", angle_1)
			angle_2 = np.arccos(np.dot(base_axis2-origin, axes2-origin) / (np.linalg.norm(base_axis2-origin) * np.linalg.norm(axes2-origin)))
			print("\n Angle 2:", angle_2)
			angle_3 = np.arccos(np.dot(base_axis3-origin, axes3-origin) / (np.linalg.norm(base_axis3-origin) * np.linalg.norm(axes3-origin)))
			print("\n Angle 3:", angle_3)

			ax.quiver(origin[0], origin[1], origin[2], axes3[0], axes3[1], axes3[2])
			ax.plot_surface(X, Y, Z)
			ax.plot(x_coords, y_coords, z_coords)
			set_axes_equal(ax)
			#plt.show()

		if gesture:
			print(gesture)
		  
		num_frames += 1
		if (time.time() - start_time) > fps_calc_window:
			fps = num_frames / fps_calc_window
			num_frames = 0
			start_time = time.time()        

		print(f"FPS: {fps}")

		# Flip the image horizontally for a selfie-view display.
		flipped = cv2.flip(processed_img, 1)
		cv2.imshow('MediaPipe Hands', flipped)
		if cv2.waitKey(5) & 0xFF == 27:
			break

finally:
	pipeline.stop()

