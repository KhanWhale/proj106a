# usr/bin/env python3
import os
import sys
import time
import cv2
import numpy as np
import mediapipe as mp
import hand_detection
import pyrealsense2 as rs
from scipy.linalg import lstsq
import matplotlib.pyplot as plt
import hand_orientation

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

hand_detection.init()
hand_orientation.init()
print("init done")

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

		processed_img, hand_landmarks, gesture = hand_detection.process(color_image)
		if hand_landmarks:
			landmarks = hand_landmarks.landmark

			for i in range(21): # keypoint in mp.solutions.hands.HandLandmark
				pixel_x, pixel_y = landmarks[i].x * image_width, landmarks[i].y * image_height
				rounded_x, rounded_y = round(pixel_x), round(pixel_y)
				depth = 0
				if rounded_x >= 0 and rounded_x < 480 and rounded_y >= 0 and rounded_y < 480:
					depth = depth_image[rounded_y][rounded_x]
				x_coords = np.append(x_coords, hand_landmarks.landmark[i].x)
				y_coords = np.append(y_coords, hand_landmarks.landmark[i].y)
				z_coords = np.append(z_coords, hand_landmarks.landmark[i].z)

				print(f"{mp.solutions.hands.HandLandmark(i)}: (X: {pixel_x}, Y: {pixel_y}, Depth: {depth})")
			print("\n\n")

			#z_coords[z_coords == 0] = np.mean(z_coords)
			#z_coords = z_coords

			angle_1, angle_2, angle_3 = hand_orientation.get_angles(x_coords, y_coords, z_coords)
			print("\n Angle 1:", angle_1)
			print("\n Angle 2:", angle_2)
			print("\n Angle 3:", angle_3)
			print("\n\n")

			os.system('clear')


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

