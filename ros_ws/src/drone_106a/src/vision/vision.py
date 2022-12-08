#!/usr/bin/env python3

# Send req to vision-manager-startup with startupCheck.srv 
# Publish handState on topic handState

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
from graph import *
import rospy
from drone_106a.msg import handState
from drone_106a.srv import startupCheck

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

# FPS calculation initialization
start_time = time.time()
fps_calc_window = 1
num_frames = 0
fps = 0
base_frame = False

rospy.init_node('vision_node', anonymous=True)
pub = rospy.Publisher('handState', handState, queue_size=50)
handstate_msg = handState()

def visionLoop(breakOnHand):
	rospy.loginfo(f"Break ON Hand : {breakOnHand}")
	global base_frame, num_frames, start_time

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
				depths = []
				for i in range(21): # keypoint in mp.solutions.hands.HandLandmark
					pixel_x, pixel_y = landmarks[i].x * image_width, landmarks[i].y * image_height
					rounded_x, rounded_y = round(pixel_x), round(pixel_y)
					depth = 0
					if rounded_x >= 0 and rounded_x < 480 and rounded_y >= 0 and rounded_y < 480:
						depth = depth_image[rounded_y][rounded_x]
					x_coords = np.append(x_coords, hand_landmarks.landmark[i].x)
					y_coords = np.append(y_coords, hand_landmarks.landmark[i].y)
					z_coords = np.append(z_coords, hand_landmarks.landmark[i].z)
					depths = np.append(depths, depth)
					print(f"{mp.solutions.hands.HandLandmark(i)}: (X: {pixel_x}, Y: {pixel_y}, Depth: {depth})")
				print("\n\n")

				depths[depths == 0] = np.mean(depths)
				depths[depths == None] = np.mean(depths)
				#z_coords = z_coords

				angle_1, angle_2, angle_3 = hand_orientation.get_angles(x_coords, y_coords, z_coords)
				mean_depth = np.mean(depths)
				print("\n Angle 1 (Yaw):", angle_1)
				print("\n Angle 2 (Pitch):", angle_2)
				print("\n Angle 3 (Roll):", angle_3)
				print("\n Mean Depth:", mean_depth)
				print("\n\n")

				if breakOnHand:
					return

				handstate_msg.roll = angle_1
				handstate_msg.pitch = angle_2
				handstate_msg.yaw = angle_3
				handstate_msg.height = mean_depth
				handstate_msg.gesture = 0 if not gesture else gesture.value
				pub.publish(handstate_msg)
				rospy.loginfo(str(handstate_msg))

			if gesture:
				print(gesture)
			
			num_frames += 1
			if (time.time() - start_time) > fps_calc_window:
				fps = num_frames / fps_calc_window
				num_frames = 0
				start_time = time.time()        

			#print(f"FPS: {fps}")
					#os.system('clear')
			# Flip the image horizontally for a selfie-view display.
			flipped = cv2.flip(processed_img, 1)
			cv2.imshow('MediaPipe Hands', flipped)
			if cv2.waitKey(5) & 0xFF == 27:
				break

	finally:
		if not breakOnHand:
			pipeline.stop()


visionLoop(True)

try:
	rospy.wait_for_service('vision_manager_startup')
	srvPrxy = rospy.ServiceProxy('vision_manager_startup', startupCheck)
	ack = srvPrxy(True)
	if not ack:
		pipeline.stop()
		exit(1)
except rospy.ServiceException as e:
	print(f"Drone startup service failed: {e}")
	pipeline.stop()
	exit(2)

visionLoop(False)
if __name__ == "__main__":
	print("vision")

	